#pragma once
#include "../fixed_point/fixed_point_math.hpp"
#include <array>

struct color{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

using namespace fixed_point;
using fs_t = fixed<int32_t, 16>;

struct particle{
    fs_t pos;
    fs_t vel;
    fs_t acc;
    color col;
};

fs_t inline force(particle& a, particle& b){
    fs_t distance = a.pos - b.pos;
    bool negative = distance < make_fixed<int32_t, 16>(0);
    
    constexpr fs_t min_distance = 0.25_fixp_t;
    fs_t result;
    if(abs(distance) < min_distance){
        result = make_fixed<int32_t, 16>(1);
    }
    else{
        result = make_fixed<int32_t, 16>(1) / (distance * distance);
    }
    
    constexpr fs_t magic_constant = 0.003_fixp_t;
    result = magic_constant * result;
    
    if(negative) return -result;
    return result;
}

static fs_t v(fs_t m1, fs_t m2, fs_t hue) {
    if(hue < make_fixed<int32_t, 16>(0)){
        constexpr fs_t one = 1.0_fixp_t;
        hue = one - hue;
    }
    hue.v = hue.v & fs_t::frac_mask(); //"fmod"
    constexpr fs_t one_sixth = 0.16666666666_fixp_t;
    constexpr fs_t six = 6.0_fixp_t;
    if (hue < one_sixth)
        return m1 + (m2 - m1) * hue * six;
        
    constexpr fs_t half = 0.5_fixp_t;
    if (hue < half)
        return m2;
    
    constexpr fs_t two_third = 0.6666666666_fixp_t;
    if (hue < two_third)
        return m1 + (m2 - m1) * (two_third - hue) * six;
    return m1;
}

static color hls_to_rgb(fs_t h, fs_t l, fs_t s){
    if(s == 0){
        uint8_t c = (l * make_fixed<int32_t, 16>(255)).v >> 16;
        return {c,c,c};
    }
    constexpr fs_t half = 0.5_fixp_t;
    constexpr fs_t one = 1.0_fixp_t;
    constexpr fs_t two = 2.0_fixp_t;
    fs_t m2 = (l <= half) ? l * (one + s) : l + s - (l * s);
    fs_t m1 = two * l - m2;
    constexpr fs_t one_third = 0.3333333333_fixp_t;
    
    fs_t r = v(m1, m2, h + one_third);
    fs_t g = v(m1, m2, h);
    fs_t b = v(m1, m2, h - one_third);
    
    color result;
    result.r = (r * make_fixed<int32_t, 16>(255)).v >> 16;
    result.g = (g * make_fixed<int32_t, 16>(255)).v >> 16;
    result.b = (b * make_fixed<int32_t, 16>(255)).v >> 16;
    return result;
}

static fs_t rng(){
    static uint32_t x32 = 0xaabbccdd;
      x32 ^= x32 << 13;
      x32 ^= x32 >> 17;
      x32 ^= x32 << 5;
      fs_t result;
      result.v = x32 & fs_t::frac_mask();
      result.v += 1;
      return result;
}

template<size_t n_particles, size_t n_leds>
struct particle_simulation{
    std::array<color, n_leds> leds;
    std::array<particle, n_particles> particles;
    
    particle_simulation(){
        for(auto& i : leds){
            i.r = 0;
            i.g = 0;
            i.b = 0;
        }
        for(size_t i = 0; i < particles.size(); i++){
            particles[i].pos = (i * (n_leds-1))/(n_particles-1);
            particles[i].vel = 0.08_fixp_t;
            particles[i].acc = 0;
            constexpr fs_t half = 0.5_fixp_t;
            constexpr fs_t one = 1.0_fixp_t;
            particles[i].col = hls_to_rgb(rng(), half, one);
        }
    }
    
    void step(fs_t dt){
        
        for(size_t i = 1; i < particles.size()-1; i++){
            fs_t acc = 0;
            acc = acc + force(particles[i], particles[i-1]);
            acc = acc + force(particles[i], particles[i+1]);
            particles[i].acc = acc;
        }
        for(size_t i = 1; i < particles.size()-1; i++){
            particles[i].vel = particles[i].vel + particles[i].acc * dt;
            constexpr fs_t friction = 0.99999_fixp_t;
            particles[i].vel = particles[i].vel * friction;
            particles[i].pos = particles[i].pos + particles[i].vel * dt;
        }
    }
    
    void update_leds(){
        for(auto& i : leds){
            i.r = 0;
            i.g = 0;
            i.b = 0;
        }
        for(auto& p : particles){
            constexpr fs_t half = 0.5_fixp_t;
            fs_t pos_for_rounding = p.pos + half;
            int led_pos = pos_for_rounding.v >> 16;
            if(led_pos >= 0 and led_pos < static_cast<int>(n_leds)){
                leds[led_pos] = p.col;
            }
        }
    }
    
};
