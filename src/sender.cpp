#include <opencv2/opencv.hpp>

#include <uvgrtp/lib.hh> // uvgRTP 3.x

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

// -------- ProgramOptions -----------------------------------------------------
struct ProgramOptions {
    std::string input_dir = ".";
    std::string pattern   = "frame_*.png";
    std::string ip        = "239.0.2.3"; // default multicast
    uint16_t port         = 2304;
    int fps               = 30;
    int bitrate           = 1'000'000;
    int gop               = 30;
    int max_frames        = -1;  // unlimited
    bool verbose          = false;

    static void print_help(const char* argv0) {
        std::cerr <<
        "Usage: " << argv0 << " [options]\n"
        "  --input DIR           Input folder (default: .)\n"
        "  --pattern GLOB        Pattern like frame_*.png (default: frame_*.png)\n"
        "  --ip IP               Destination IP (default: 239.0.2.3)\n"
        "  --port N              Destination port (default: 2304)\n"
        "  --fps N               FPS (default: 30)\n"
        "  --bitrate N           Bitrate in bps (default: 1000000)\n"
        "  --gop N               GOP size / keyint (default: 30)\n"
        "  --max-frames N        Max frames to process (default: all)\n"
        "  --verbose | -v        Verbose logging\n"
        "  -h | --help           This help\n";
    }

    bool parse(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            auto need = [&](const char* name)->bool{
                if (i+1 >= argc) { std::cerr << name << " requires a value\n"; return false; }
                return true;
            };
            if (a == "--input")            { if (!need("--input")) return false; input_dir = argv[++i]; }
            else if (a == "--pattern")     { if (!need("--pattern")) return false; pattern = argv[++i]; }
            else if (a == "--ip")          { if (!need("--ip")) return false; ip = argv[++i]; }
            else if (a == "--port")        { if (!need("--port")) return false; port = (uint16_t)std::stoi(argv[++i]); }
            else if (a == "--fps")         { if (!need("--fps")) return false; fps = std::stoi(argv[++i]); }
            else if (a == "--bitrate")     { if (!need("--bitrate")) return false; bitrate = std::stoi(argv[++i]); }
            else if (a == "--gop")         { if (!need("--gop")) return false; gop = std::stoi(argv[++i]); }
            else if (a == "--max-frames")  { if (!need("--max-frames")) return false; max_frames = std::stoi(argv[++i]); }
            else if (a == "--verbose" || a == "-v") { verbose = true; }
            else if (a == "-h" || a == "--help") { print_help(argv[0]); return false; }
            else { std::cerr << "Unknown arg: " << a << "\n"; print_help(argv[0]); return false; }
        }
        if (fps <= 0) { std::cerr << "fps must be > 0\n"; return false; }
        if (bitrate <= 0) { std::cerr << "bitrate must be > 0\n"; return false; }
        if (gop <= 0) { std::cerr << "gop must be > 0\n"; return false; }
        return true;
    }
};

// -------- Small utilities ----------------------------------------------------
static std::regex glob_to_regex(const std::string& pat) {
    std::string rx;
    rx.reserve(pat.size()*2);
    for (char c : pat) {
        switch (c) {
            case '*': rx += ".*"; break;
            case '?': rx += '.';  break;
            case '.': rx += "\\."; break;
            default:  rx += c;    break;
        }
    }
    return std::regex("^" + rx + "$", std::regex::ECMAScript | std::regex::icase);
}

static bool looks_annexb(const uint8_t* p, int n) {
    if (n >= 4 && p[0]==0 && p[1]==0 && p[2]==0 && p[3]==1) return true;
    if (n >= 3 && p[0]==0 && p[1]==0 && p[2]==1) return true;
    return false;
}

// -------- RAII for FFmpeg pointers ------------------------------------------
struct AVCodecContextDel { void operator()(AVCodecContext* c) const { if (c) avcodec_free_context(&c); } };
struct AVFrameDel        { void operator()(AVFrame* f)        const { if (f) av_frame_free(&f); } };
struct AVPacketDel       { void operator()(AVPacket* p)       const { if (p) av_packet_free(&p); } };
struct SwsContextDel     { void operator()(SwsContext* s)     const { if (s) sws_freeContext(s); } };
struct AVBSFContextDel   { void operator()(AVBSFContext* b)   const { if (b) av_bsf_free(&b); } };

// -------- AVCC (no-BSF) helpers --------------------------------------------
struct SPSPPS {
    std::vector<uint8_t> sps, pps;
    int nal_len_size = 4; // default
};

static SPSPPS parse_avcc_extradata(const AVCodecContext* c) {
    SPSPPS out;
    if (!c || !c->extradata || c->extradata_size < 7) return out;
    const uint8_t* e = c->extradata;
    int n = c->extradata_size;
    if (e[0] != 1) return out; // not avcC
    out.nal_len_size = (e[4] & 0x3) + 1;
    int pos = 5;
    int num_sps = e[pos++] & 0x1F;
    for (int i = 0; i < num_sps; ++i) {
        if (pos+2 > n) break;
        int len = (e[pos]<<8) | e[pos+1]; pos+=2;
        if (pos+len > n) break;
        out.sps.assign(e+pos, e+pos+len); pos+=len;
    }
    if (pos >= n) return out;
    int num_pps = e[pos++];
    for (int i = 0; i < num_pps; ++i) {
        if (pos+2 > n) break;
        int len = (e[pos]<<8) | e[pos+1]; pos+=2;
        if (pos+len > n) break;
        out.pps.assign(e+pos, e+pos+len); pos+=len;
    }
    return out;
}

static void avcc_payload_to_annexb(const uint8_t* in, int in_size, int nal_len_size, std::vector<uint8_t>& out) {
    out.clear();
    int i = 0;
    auto push_sc = [&]{ out.insert(out.end(), {0,0,0,1}); };
    while (i + nal_len_size <= in_size) {
        uint32_t nal_len = 0;
        for (int k = 0; k < nal_len_size; ++k) nal_len = (nal_len<<8) | in[i+k];
        i += nal_len_size;
        if (nal_len == 0 || i + (int)nal_len > in_size) break;
        push_sc();
        out.insert(out.end(), in + i, in + i + nal_len);
        i += nal_len;
    }
}

static void prepend_sps_pps(std::vector<uint8_t>& au, const SPSPPS& hp) {
    if (hp.sps.empty() || hp.pps.empty()) return;
    std::vector<uint8_t> out;
    const uint8_t sc[4] = {0,0,0,1};
    out.insert(out.end(), sc, sc+4); out.insert(out.end(), hp.sps.begin(), hp.sps.end());
    out.insert(out.end(), sc, sc+4); out.insert(out.end(), hp.pps.begin(), hp.pps.end());
    out.insert(out.end(), au.begin(), au.end());
    au.swap(out);
}

// -------- Encoded AU container ----------------------------------------------
struct EncodedAU {
    std::vector<uint8_t> bytes; // Annex-B access unit
    int64_t pts = 0;            // in encoder time_base ticks
    bool key = false;
};

// -------- Encoder (OpenCV BGR -> H.264 Annex-B) ------------------------------
class Encoder {
public:
    Encoder() = default;
    ~Encoder() = default;

    bool init(int w, int h, int fps, int bitrate, int gop, bool verbose) {
        verbose_ = verbose;

        const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
        if (!codec) codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec) { std::cerr << "No H.264 encoder found\n"; return false; }

        ctx_.reset(avcodec_alloc_context3(codec));
        if (!ctx_) { std::cerr << "avcodec_alloc_context3 failed\n"; return false; }

        ctx_->width  = w;
        ctx_->height = h;
        ctx_->time_base = AVRational{1, fps};
        ctx_->framerate = AVRational{fps, 1};
        ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
        ctx_->gop_size = gop;
        ctx_->max_b_frames = 0;
        ctx_->bit_rate = bitrate;

        // In-band headers for RTP
        ctx_->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;

        // x264 knobs if available
        av_opt_set(ctx_->priv_data, "preset", "veryfast", 0);
        av_opt_set(ctx_->priv_data, "tune", "zerolatency", 0);
        av_opt_set(ctx_->priv_data, "repeat-headers", "1", 0);

        if (avcodec_open2(ctx_.get(), codec, nullptr) < 0) {
            std::cerr << "avcodec_open2 failed\n";
            return false;
        }

        frame_.reset(av_frame_alloc());
        frame_->format = ctx_->pix_fmt;
        frame_->width  = w;
        frame_->height = h;
        if (av_frame_get_buffer(frame_.get(), 32) < 0) {
            std::cerr << "av_frame_get_buffer failed\n";
            return false;
        }

        sws_.reset(sws_getContext(
            w, h, AV_PIX_FMT_BGR24,
            w, h, AV_PIX_FMT_YUV420P,
            SWS_BILINEAR, nullptr, nullptr, nullptr
        ));
        if (!sws_) { std::cerr << "sws_getContext failed\n"; return false; }

        // Try BSF first
        const AVBitStreamFilter* f = av_bsf_get_by_name("h264_mp4toannexb");
        if (f) {
            AVBSFContext* raw = nullptr;
            if (av_bsf_alloc(f, &raw) < 0) { std::cerr << "av_bsf_alloc failed\n"; return false; }
            bsf_.reset(raw);
            if (avcodec_parameters_from_context(bsf_->par_in, ctx_.get()) < 0) {
                std::cerr << "avcodec_parameters_from_context failed\n"; return false; }
            if (av_bsf_init(bsf_.get()) < 0) { std::cerr << "av_bsf_init failed\n"; return false; }
            use_bsf_ = true;
        } else {
            // No BSF: parse avcC to prepare manual conversion
            hp_ = parse_avcc_extradata(ctx_.get());
            use_bsf_ = false;
            if (verbose_) std::cerr << "[warn] BSF not found; using AVCC->AnnexB fallback\n";
        }

        return true;
    }

    // Encode preprocessed BGR frame (size exactly w x h, 3-ch)
    bool encode(const cv::Mat& bgr, std::vector<EncodedAU>& out_collector, int64_t pts) {
        if (bgr.cols != ctx_->width || bgr.rows != ctx_->height || bgr.channels() != 3) {
            std::cerr << "encode: unexpected frame shape\n"; return false;
        }

        if (av_frame_make_writable(frame_.get()) < 0) { std::cerr << "frame not writable\n"; return false; }

        const uint8_t* src[1] = { bgr.data };
        int src_stride[1] = { static_cast<int>(bgr.step[0]) };
        sws_scale(sws_.get(), src, src_stride, 0, ctx_->height, frame_->data, frame_->linesize);

        frame_->pts = pts;

        if (int e = avcodec_send_frame(ctx_.get(), frame_.get()); e < 0) {
            std::cerr << "avcodec_send_frame: " << e << "\n"; return false;
        }

        while (true) {
            std::unique_ptr<AVPacket, AVPacketDel> pkt(av_packet_alloc());
            int r = avcodec_receive_packet(ctx_.get(), pkt.get());
            if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) break;
            if (r < 0) { std::cerr << "avcodec_receive_packet: " << r << "\n"; return false; }

            if (use_bsf_) {
                if (av_bsf_send_packet(bsf_.get(), pkt.get()) < 0) { std::cerr << "bsf_send failed\n"; return false; }
                av_packet_unref(pkt.get());

                while (true) {
                    std::unique_ptr<AVPacket, AVPacketDel> out(av_packet_alloc());
                    int br = av_bsf_receive_packet(bsf_.get(), out.get());
                    if (br == AVERROR(EAGAIN) || br == AVERROR_EOF) break;
                    if (br < 0) { std::cerr << "bsf_receive failed\n"; return false; }

                    EncodedAU au;
                    au.bytes.assign(out->data, out->data + out->size);
                    au.pts = out->pts; // in {1,fps}
                    au.key = (out->flags & AV_PKT_FLAG_KEY) != 0;
                    out_collector.emplace_back(std::move(au));
                }
            } else {
                // Fallback: AVCC->AnnexB + prepend headers on keyframes if needed
                EncodedAU au;
                if (looks_annexb(pkt->data, pkt->size)) {
                    au.bytes.assign(pkt->data, pkt->data + pkt->size);
                } else {
                    avcc_payload_to_annexb(pkt->data, pkt->size, hp_.nal_len_size, au.bytes);
                }
                if ((pkt->flags & AV_PKT_FLAG_KEY) && !(hp_.sps.empty() || hp_.pps.empty())) {
                    prepend_sps_pps(au.bytes, hp_);
                }
                au.pts = pkt->pts;
                au.key = (pkt->flags & AV_PKT_FLAG_KEY) != 0;
                out_collector.emplace_back(std::move(au));
            }
        }
        return true;
    }

    bool flush(std::vector<EncodedAU>& out_collector) {
        if (avcodec_send_frame(ctx_.get(), nullptr) < 0) return false;
        while (true) {
            std::unique_ptr<AVPacket, AVPacketDel> pkt(av_packet_alloc());
            int r = avcodec_receive_packet(ctx_.get(), pkt.get());
            if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) break;
            if (r < 0) { std::cerr << "flush receive_packet: " << r << "\n"; return false; }

            if (use_bsf_) {
                if (av_bsf_send_packet(bsf_.get(), pkt.get()) < 0) return false;
                av_packet_unref(pkt.get());
                while (true) {
                    std::unique_ptr<AVPacket, AVPacketDel> out(av_packet_alloc());
                    int br = av_bsf_receive_packet(bsf_.get(), out.get());
                    if (br == AVERROR(EAGAIN) || br == AVERROR_EOF) break;
                    if (br < 0) return false;

                    EncodedAU au;
                    au.bytes.assign(out->data, out->data + out->size);
                    au.pts = out->pts;
                    au.key = (out->flags & AV_PKT_FLAG_KEY) != 0;
                    out_collector.emplace_back(std::move(au));
                }
            } else {
                EncodedAU au;
                if (looks_annexb(pkt->data, pkt->size)) au.bytes.assign(pkt->data, pkt->data + pkt->size);
                else avcc_payload_to_annexb(pkt->data, pkt->size, hp_.nal_len_size, au.bytes);
                if ((pkt->flags & AV_PKT_FLAG_KEY) && !(hp_.sps.empty() || hp_.pps.empty()))
                    prepend_sps_pps(au.bytes, hp_);
                au.pts = pkt->pts;
                au.key = (pkt->flags & AV_PKT_FLAG_KEY) != 0;
                out_collector.emplace_back(std::move(au));
            }
        }
        return true;
    }

    AVRational time_base() const { return ctx_->time_base; }
    int width() const { return ctx_->width; }
    int height() const { return ctx_->height; }

private:
    bool verbose_ = false;
    std::unique_ptr<AVCodecContext, AVCodecContextDel> ctx_;
    std::unique_ptr<AVFrame, AVFrameDel> frame_;
    std::unique_ptr<SwsContext, SwsContextDel> sws_;
    std::unique_ptr<AVBSFContext, AVBSFContextDel> bsf_;
    bool use_bsf_ = false;
    SPSPPS hp_;
};

// -------- Load & preprocess images ------------------------------------------
static bool load_and_preprocess(const ProgramOptions& opt,
                                std::vector<cv::Mat>& frames_out,
                                int& W, int& H)
{
    frames_out.clear();

    if (!fs::exists(opt.input_dir) || !fs::is_directory(opt.input_dir)) {
        std::cerr << "Input dir doesn't exist or not a directory: " << opt.input_dir << "\n";
        return false;
    }

    std::regex rx = glob_to_regex(opt.pattern);
    std::vector<fs::path> list;
    for (auto& p : fs::directory_iterator(opt.input_dir)) {
        if (!p.is_regular_file()) continue;
        auto name = p.path().filename().string();
        if (std::regex_match(name, rx)) list.push_back(p.path());
    }
    if (list.empty()) {
        std::cerr << "No files matching pattern in " << opt.input_dir << "\n";
        return false;
    }

    std::sort(list.begin(), list.end());

    int count = 0;
    cv::Mat first;
    for (auto& path : list) {
        if (opt.max_frames >= 0 && count >= opt.max_frames) break;

        cv::Mat img = cv::imread(path.string(), cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Failed to read: " << path << "\n";
            return false;
        }
        if (first.empty()) {
            first = img;
            W = first.cols; H = first.rows;
        }

        if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        else if (img.channels() == 4) cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

        if (img.cols != W || img.rows != H) {
            cv::Mat resized;
            cv::resize(img, resized, cv::Size(W, H), 0, 0, cv::INTER_AREA);
            img = resized;
        }
        frames_out.push_back(std::move(img));
        ++count;
    }

    if (opt.verbose) {
        std::cerr << "Loaded " << frames_out.size() << " frames (" << W << "x" << H << ")\n";
    }
    return !frames_out.empty();
}

// -------- RTP sender (uvgRTP) -----------------------------------------------
static bool send_rtsp_annexb_sequence(const ProgramOptions& opt,
                                      const std::vector<EncodedAU>& aus,
                                      AVRational enc_tb)
{
    if (aus.empty()) {
        std::cerr << "No encoded frames to send\n";
        return false;
    }

    // uvgRTP: context -> session -> stream
    uvgrtp::context ctx;
    std::unique_ptr<uvgrtp::session, std::function<void(uvgrtp::session*)>> sess(
        ctx.create_session(opt.ip),
        [&](uvgrtp::session* s){ if (s) ctx.destroy_session(s); }
    );
    if (!sess) { std::cerr << "create_session failed\n"; return false; }

    // Unidirectional send-only stream.
    // Single-port variant interprets 'port' as remote if RCE_SEND_ONLY is set.
    std::unique_ptr<uvgrtp::media_stream, std::function<void(uvgrtp::media_stream*)>> strm(
        sess->create_stream(opt.port, RTP_FORMAT_H264, RCE_SEND_ONLY),
        [&](uvgrtp::media_stream* m){ if (m) sess->destroy_stream(m); }
    );
    if (!strm) { std::cerr << "create_stream failed\n"; return false; }

    // Configure 90 kHz clock and a reasonable MTU; disable H26x AP aggregation for camera-like behavior.
    constexpr int DEFAULT_CLOCK = 90000;
    strm->configure_ctx(RCC_CLOCK_RATE, DEFAULT_CLOCK);                 // 90 kHz clock
    strm->configure_ctx(RCC_MTU_SIZE,   1200);                          // FU-A fragmentation visible
    strm->configure_ctx(RCC_MULTICAST_TTL, 1);                          // if multicast
    const int rtp_flags = RTP_H26X_DO_NOT_AGGR;                          // no AP/STAP (H264)  (doc: util.hh)

    // Pace based on FPS; map PTS (enc_tb) -> 90k RTP timestamp
    const auto frame_period = std::chrono::duration<double>(1.0 / double(opt.fps));
    auto t0 = std::chrono::steady_clock::now();
    auto next_deadline = t0;

    int sent = 0;
    for (const auto& au : aus) {
        // Compute RTP timestamp from encoder PTS
        int64_t ts90 = av_rescale_q(au.pts, enc_tb, AVRational{1, DEFAULT_CLOCK});

        // Sleep until next frame deadline
        next_deadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
        std::this_thread::sleep_until(next_deadline);

        // Send one Access Unit per push_frame; uvgRTP fragments as needed (SCL enabled by default)
        rtp_error_t er = strm->push_frame(const_cast<uint8_t*>(au.bytes.data()),
                                          au.bytes.size(),
                                          (uint32_t)ts90,
                                          rtp_flags);
        if (er != RTP_OK) {
            std::cerr << "push_frame error " << er << " at index " << sent << "\n";
            return false;
        }

        if (opt.verbose) {
            std::cerr << "sent idx=" << sent
                      << " pts=" << au.pts
                      << " ts90=" << ts90
                      << " size=" << au.bytes.size()
                      << (au.key ? " [IDR]\n" : "\n");
        }
        ++sent;
    }

    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cerr << "Done. Sent " << sent << " frames in " << elapsed << " s (target ~"
              << aus.size() / double(opt.fps) << " s)\n";
    return true;
}

// -------- main ---------------------------------------------------------------
int main(int argc, char** argv) {
    av_log_set_level(AV_LOG_ERROR); // keep noise down

    ProgramOptions opt;
    if (!opt.parse(argc, argv)) return 1;

    // 1) Load & preprocess
    std::vector<cv::Mat> frames;
    int W = 0, H = 0;
    if (!load_and_preprocess(opt, frames, W, H)) return 2;

    // 2) Init encoder
    Encoder enc;
    if (!enc.init(W, H, opt.fps, opt.bitrate, opt.gop, opt.verbose)) return 3;

    // 3) Encode all frames to Annex-B Access Units
    std::vector<EncodedAU> aus;
    aus.reserve(frames.size() + 8);

    int64_t pts = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < frames.size(); ++i) {
        if (!enc.encode(frames[i], aus, pts++)) {
            std::cerr << "encode failed at " << i << "\n";
            return 4;
        }
    }
    if (!enc.flush(aus)) {
        std::cerr << "flush failed\n";
        return 5;
    }
    auto enc_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (opt.verbose) {
        int idr = 0;
        for (auto& au : aus) if (au.key) ++idr;
        std::cerr << "Encoded " << aus.size() << " AUs (" << idr << " IDR) in "
                  << enc_elapsed << " s\n";
    }

    // 4) Send at FPS over RTP (uvgRTP)
    if (!send_rtsp_annexb_sequence(opt, aus, enc.time_base())) return 6;

    return 0;
}
