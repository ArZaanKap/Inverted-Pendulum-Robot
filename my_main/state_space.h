#pragma once
#include <Arduino.h>

// Physical parameters (from dynamics.py)
constexpr float M_cart_total  = 1.593f; // REMEASURE
constexpr float m_rod   = 0.043f;
constexpr float m_tip   = 0.050f;
constexpr float M_cart  = M_cart_total - m_rod - m_tip; // 0.828
constexpr float m_pend  = m_rod + m_tip;
constexpr float L_rod   = 0.6f;
constexpr float l_com   = (m_rod * L_rod / 2.0f + m_tip * L_rod) / m_pend;
constexpr float I_pivot = (1.0f/12.0f)*m_rod*L_rod*L_rod
                                + m_rod*(L_rod/2.0f)*(L_rod/2.0f)
                                + m_tip*L_rod*L_rod;

constexpr float b_x     = 0.1f;
constexpr float b_theta = 0.004320f;
constexpr float g       = 9.81f;
constexpr float M_t     = M_cart + m_pend;
constexpr float ml      = m_pend * l_com;
constexpr float D_denom = M_t * I_pivot - ml * ml;

// Non-zero elements of linearised continuous A matrix
constexpr float A22 = -(I_pivot * b_x) / D_denom;  // Renamed from A21 to correctly reflect row 2, col 2
constexpr float A23 = -(ml * ml * g)   / D_denom;
constexpr float A24 =  (ml * b_theta)  / D_denom;
constexpr float A42 =  (ml * b_x)      / D_denom;
constexpr float A43 =  (M_t * ml * g)  / D_denom;
constexpr float A44 = -(M_t * b_theta) / D_denom;

// Input matrix B elements (1-indexed rows)
constexpr float B2  =  I_pivot / D_denom;
constexpr float B4  = -ml      / D_denom;


// ADDED - NEW

// Discrete-time A and B (Euler discretisation at dt = 0.01s, 100Hz)
// A_d = I + A_c * dt
// B_d = B_c * dt
constexpr float dt = 0.0025f; // IMPORTANT - in seconds (400Hz)

constexpr float A[4][4] = {
  { 1,           dt,          0,          0   },
  { 0,  1 + A22*dt,    A23*dt,    A24*dt   }, // Updated to use A22
  { 0,           0,          1,          dt  },
  { 0,     A42*dt,       A43*dt,  1+A44*dt   } // FIXED: Removed "1+" from A43
};

constexpr float B[4] = { 0, B2*dt, 0, B4*dt };

