#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <utility>

class HilbertCurve2D {
private:
    uint32_t order;


    void rot(uint32_t n, uint32_t& x, uint32_t& y, uint32_t rx, uint32_t ry) {
        if (ry == 0) {
            if (rx == 1) {
                x = n - 1 - x;
                y = n - 1 - y;
            }

            
            std::swap(x, y);
        }
    }

public:
    HilbertCurve2D(uint32_t order) : order(order) {}

    
    uint32_t encode(uint32_t x, uint32_t y) {
        uint32_t hilbertValue = 0;
        for (uint32_t s = (1 << (order - 1)); s > 0; s >>= 1) {
            uint32_t rx = (x & s) > 0;
            uint32_t ry = (y & s) > 0;
            hilbertValue += s * s * ((3 * rx) ^ ry);
            rot(s, x, y, rx, ry);
        }
        return hilbertValue;
    }

 
    std::pair<uint32_t, uint32_t> decode(uint32_t hilbertValue) {
        uint32_t x = 0;
        uint32_t y = 0;

        for (uint32_t s = 1; s < (1 << order); s <<= 1) {
            uint32_t rx = (hilbertValue & 2) > 0;
            uint32_t ry = ((hilbertValue ^ rx) & 1) > 0;
            rot(s, x, y, rx, ry);
            x += s * rx;
            y += s * ry;
            hilbertValue >>= 2;
        }

        return std::make_pair(x, y);
    }
};