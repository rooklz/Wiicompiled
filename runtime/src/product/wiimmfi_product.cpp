#include "runtime_product.h"

namespace RuntimeProduct {

const Descriptor& Active() noexcept {
    static constexpr Descriptor descriptor{
        Kind::Wiimmfi,
        "Mario Kart Wii (Wiimmfi)",
    };
    return descriptor;
}

} // namespace RuntimeProduct
