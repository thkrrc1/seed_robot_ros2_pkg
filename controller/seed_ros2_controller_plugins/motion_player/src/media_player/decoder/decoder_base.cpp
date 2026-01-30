#include "decoder_base.hpp"

namespace motion_editor2 {
#ifdef HW_ACCEL
AVPixelFormat DecoderBase::hw_pix_fmt = AV_PIX_FMT_NONE;
AVBufferRef *DecoderBase::hw_device_ctx = nullptr;
#endif

}
