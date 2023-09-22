#ifndef MYHASH_H_INCLUDED
#define MYHASH_H_INCLUDED

#include <Eigen/Dense>

namespace MyHash {
    struct EigenVectorXfHash {
        std::size_t operator()(const Eigen::VectorXf& vector) const {
            std::size_t seed = vector.size();
            for (int i = 0; i < vector.size(); ++i) {
                auto value = std::hash<float>{}(vector(i));
                seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}


#endif