/**
 * @file eigen_matrix_utils.hpp
 * @brief Optimized matrix operations using Eigen for embedded systems
 * 
 * Replaces manual matrix operations with Eigen for:
 * - SIMD vectorization (NEON on ARM, SSE on x86)
 * - Cache-friendly memory layout
 * - Compile-time optimization
 * - Numerical stability
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <cmath>
#include <array>

namespace radarprocessing {

// ─────────────────────────────────────────────────────────────────────────────
// Type Definitions - Fixed-size matrices for compile-time optimization
// ─────────────────────────────────────────────────────────────────────────────

// State: [px, py, vx, vy]^T - 4D state vector
using Vector4f = Eigen::Vector4f;
using Matrix4f = Eigen::Matrix4f;

// Measurement: [mx, my]^T - 2D measurement vector  
using Vector2f = Eigen::Vector2f;
using Matrix2f = Eigen::Matrix2f;

// Observation matrix H (2×4): measures position only
using Matrix2x4f = Eigen::Matrix<float, 2, 4>;

// Kalman gain K (4×2)
using Matrix4x2f = Eigen::Matrix<float, 4, 2>;

// ─────────────────────────────────────────────────────────────────────────────
// Compatibility Layer - Maintains API for existing code
// ─────────────────────────────────────────────────────────────────────────────

// Legacy type for backward compatibility
using M4 = std::array<float, 16>;
using M2 = std::array<float, 4>;

// Convert between Eigen and legacy array format
inline Matrix4f fromM4(const M4& m) {
    Matrix4f mat;
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            mat(i, j) = m[i*4 + j];
    return mat;
}

inline M4 toM4(const Matrix4f& mat) {
    M4 m;
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            m[i*4 + j] = mat(i, j);
    return m;
}

inline Matrix2f fromM2(const M2& m) {
    Matrix2f mat;
    mat << m[0], m[1], m[2], m[3];
    return mat;
}

inline M2 toM2(const Matrix2f& mat) {
    M2 m;
    m[0] = mat(0,0); m[1] = mat(0,1);
    m[2] = mat(1,0); m[3] = mat(1,1);
    return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Optimized EKF Operations
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief EKF prediction step - optimized with Eigen
 * 
 * x = F * x
 * P = F * P * F^T + Q
 * 
 * Uses .noalias() to avoid temporary allocation
 */
inline void ekfPredict(Vector4f& x, Matrix4f& P, 
                       const Matrix4f& F, const Matrix4f& Q) {
    // State prediction
    x.noalias() = F * x;
    
    // Covariance prediction - Joseph form for numerical stability
    Matrix4f temp = F * P;
    P.noalias() = temp * F.transpose();
    P.noalias() += Q;
    
    // Ensure symmetry (numerical stability)
    P = 0.5f * (P + P.transpose());
}

/**
 * @brief EKF update step - optimized with Eigen
 * 
 * Innovation: y = z - H * x
 * Innovation covariance: S = H * P * H^T + R
 * Kalman gain: K = P * H^T * S^{-1}
 * State update: x = x + K * y
 * Covariance update: P = (I - K*H) * P
 * 
 * Uses Cholesky decomposition for stable inverse
 */
inline void ekfUpdate(Vector4f& x, Matrix4f& P,
                     const Vector2f& z, const Matrix2x4f& H, 
                     const Matrix2f& R) {
    // Innovation
    Vector2f y = z - H * x;
    
    // Innovation covariance
    Matrix2f S = H * P * H.transpose() + R;
    
    // Kalman gain - use Cholesky for numerical stability
    Eigen::LLT<Matrix2f> llt(S);
    if(llt.info() != Eigen::Success) {
        // Fallback to direct inverse if Cholesky fails
        Matrix4x2f K = P * H.transpose() * S.inverse();
        x.noalias() += K * y;
        P.noalias() -= K * H * P;
    } else {
        Matrix4x2f K = P * H.transpose() * llt.solve(Matrix2f::Identity());
        
        // State update
        x.noalias() += K * y;
        
        // Covariance update - Joseph form for numerical stability
        Matrix4f I_KH = Matrix4f::Identity() - K * H;
        Matrix4f temp = I_KH * P;
        P.noalias() = temp * I_KH.transpose();
        P.noalias() += K * R * K.transpose();
    }
    
    // Ensure symmetry
    P = 0.5f * (P + P.transpose());
}

/**
 * @brief Compute Mahalanobis distance squared
 * 
 * d^2 = (z - H*x)^T * S^{-1} * (z - H*x)
 * where S = H * P * H^T + R
 * 
 * Optimized using Cholesky decomposition
 */
inline float mahalanobisDistanceSq(const Vector4f& x, const Matrix4f& P,
                                   const Vector2f& z, const Matrix2x4f& H,
                                   const Matrix2f& R) {
    // Innovation
    Vector2f y = z - H * x;
    
    // Innovation covariance
    Matrix2f S = H * P * H.transpose() + R;
    
    // Compute y^T * S^{-1} * y using Cholesky
    Eigen::LLT<Matrix2f> llt(S);
    if(llt.info() != Eigen::Success) {
        // Fallback
        return y.dot(S.inverse() * y);
    }
    
    Vector2f temp = llt.solve(y);
    return y.dot(temp);
}

/**
 * @brief Simplified Mahalanobis distance for position-only measurement
 * 
 * Assumes H = [I_2x2, 0_2x2] (position-only observation)
 * This allows optimization: skip matrix multiply
 */
inline float mahalanobisDistanceSqPos(const Vector4f& x, const Matrix4f& P,
                                      float mx, float my, float r_noise) {
    // Innovation (position difference)
    float dx = mx - x(0);
    float dy = my - x(1);
    
    // Innovation covariance (top-left 2x2 of P + R)
    Matrix2f S;
    S(0,0) = P(0,0) + r_noise * r_noise;
    S(0,1) = P(0,1);
    S(1,0) = P(1,0);
    S(1,1) = P(1,1) + r_noise * r_noise;
    
    // Compute inverse - 2x2 is cheap
    float det = S(0,0) * S(1,1) - S(0,1) * S(1,0);
    if(std::fabs(det) < 1e-12f) return 1e9f;
    
    float inv_det = 1.0f / det;
    float Si00 = S(1,1) * inv_det;
    float Si01 = -S(0,1) * inv_det;
    float Si11 = S(0,0) * inv_det;
    
    // Mahalanobis distance squared
    return dx * (Si00 * dx + Si01 * dy) + dy * (Si01 * dx + Si11 * dy);
}

/**
 * @brief Create state transition matrix F for constant velocity model
 * 
 * F = [1, 0, dt, 0 ]
 *     [0, 1, 0,  dt]
 *     [0, 0, 1,  0 ]
 *     [0, 0, 0,  1 ]
 */
inline Matrix4f createStateTransition(float dt) {
    Matrix4f F = Matrix4f::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
}

/**
 * @brief Create process noise covariance Q (Continuous White Noise Acceleration)
 * 
 * Q = σ^2 * [dt^4/4,  0,      dt^3/2,  0     ]
 *           [0,       dt^4/4, 0,       dt^3/2]
 *           [dt^3/2,  0,      dt^2,    0     ]
 *           [0,       dt^3/2, 0,       dt^2  ]
 */
inline Matrix4f createProcessNoise(float dt, float sigma) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float s2 = sigma * sigma;
    
    Matrix4f Q;
    Q << dt4/4.0f*s2,  0.0f,         dt3/2.0f*s2,  0.0f,
         0.0f,         dt4/4.0f*s2,  0.0f,         dt3/2.0f*s2,
         dt3/2.0f*s2,  0.0f,         dt2*s2,       0.0f,
         0.0f,         dt3/2.0f*s2,  0.0f,         dt2*s2;
    
    return Q;
}

/**
 * @brief Create observation matrix H (position-only)
 * 
 * H = [1, 0, 0, 0]
 *     [0, 1, 0, 0]
 */
inline Matrix2x4f createObservationMatrix() {
    Matrix2x4f H = Matrix2x4f::Zero();
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    return H;
}

/**
 * @brief Create measurement noise covariance R
 * 
 * R = [σ^2,  0  ]
 *     [0,    σ^2]
 */
inline Matrix2f createMeasurementNoise(float sigma) {
    float s2 = sigma * sigma;
    Matrix2f R;
    R << s2, 0.0f,
         0.0f, s2;
    return R;
}

/**
 * @brief Create initial covariance matrix
 */
inline Matrix4f createInitialCovariance(float pos_var, float vel_var) {
    Matrix4f P = Matrix4f::Zero();
    P(0,0) = pos_var;
    P(1,1) = pos_var;
    P(2,2) = vel_var;
    P(3,3) = vel_var;
    return P;
}

// ─────────────────────────────────────────────────────────────────────────────
// Workspace for Reusable Computation (Reduce Allocations)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Pre-allocated workspace for EKF computations
 * 
 * Reuse across multiple tracks to avoid repeated allocation
 * Stack-allocated, cache-friendly
 */
struct alignas(16) EKFWorkspace {
    Matrix4f F;              // State transition
    Matrix4f Q;              // Process noise
    Matrix4f temp4;          // Temporary 4x4
    Matrix2f S;              // Innovation covariance
    Matrix2f R;              // Measurement noise
    Matrix2x4f H;            // Observation matrix
    Matrix2x4f HP;           // H * P intermediate
    Matrix4x2f K;            // Kalman gain
    Vector4f x_pred;         // Predicted state
    Vector2f innovation;     // Innovation vector
    
    // Alignment for SIMD
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace radarprocessing