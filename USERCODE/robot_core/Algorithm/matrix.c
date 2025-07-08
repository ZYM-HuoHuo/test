/**
 * @file matrix.c
 * @brief 悲 小搓一个没有硬件加速的超简单矩阵运算
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "matrix.h"
#include <math.h>
#include <string.h>

#define ZERO_QUAD(x) x._11 = x._12 = x._21 = x._22 = 0.0f

void quad_matvec_mult(quad_vec_t *A, dual_vec_t *B, dual_vec_t *C) {
    static quad_vec_t _A;
    memcpy(&_A, A, sizeof(quad_vec_t));
    static dual_vec_t _B;
    memcpy(&_B, B, sizeof(dual_vec_t));
    C->_11 = _A._11 * _B._11 + _A._12 * _B._21;
    C->_21 = _A._21 * _B._11 + _A._22 * _B._21;
}
void quad_mat_mult(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C) {
    static quad_vec_t _A;
    memcpy(&_A, A, sizeof(quad_vec_t));
    static quad_vec_t _B;
    memcpy(&_B, B, sizeof(quad_vec_t));
    C->_11 = _A._11 * _B._11 + _A._12 * _B._21;
    C->_12 = _A._11 * _B._12 + _A._12 * _B._22;
    C->_21 = _A._21 * _B._11 + _A._22 * _B._21;
    C->_22 = _A._21 * _B._12 + _A._22 * _B._22;
}
void quad_mat_add(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C) {
    C->_11 = A->_11 + B->_11;
    C->_12 = A->_12 + B->_12;
    C->_21 = A->_21 + B->_21;
    C->_22 = A->_22 + B->_22;
}
void quad_mat_sub(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C) {
    C->_11 = A->_11 - B->_11;
    C->_12 = A->_12 - B->_12;
    C->_21 = A->_21 - B->_21;
    C->_22 = A->_22 - B->_22;
}
void quad_mat_transpose(quad_vec_t *A, quad_vec_t *C) {
    static quad_vec_t _A;
    memcpy(&_A, A, sizeof(quad_vec_t));
    C->_11 = _A._11;
    C->_12 = _A._21;
    C->_21 = _A._12;
    C->_22 = _A._22;
}
bool quad_mat_inv(quad_vec_t *A, quad_vec_t *A_inv) {
    float det = A->_11 * A->_22 - A->_12 * A->_21; // 行列式
    if (det == 0) {
        return false; // 不可逆
    }
    static quad_vec_t _A;
    memcpy(&_A, A, sizeof(quad_vec_t));
    A_inv->_11 = _A._22 / det;
    A_inv->_12 = -_A._12 / det;
    A_inv->_21 = -_A._21 / det;
    A_inv->_22 = _A._11 / det;
    return true; // 成功求逆
}

/*************三维矩阵*********** */
void triple_matvec_mult(triple_mat_t *A, triple_vec_t *B, triple_vec_t *C) {
    static triple_mat_t _A;
    memcpy(&_A, A, sizeof(triple_mat_t));
    static triple_vec_t _B;
    memcpy(&_B, B, sizeof(triple_vec_t));
    C->_11 = _A._11 * _B._11 + _A._12 * _B._21 + _A._13 * _B._31;
    C->_21 = _A._21 * _B._11 + _A._22 * _B._21 + _A._23 * _B._31;
    C->_31 = _A._31 * _B._11 + _A._32 * _B._21 + _A._33 * _B._31;
}
void triple_mat_mult(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C) {
    static triple_mat_t _A;
    memcpy(&_A, A, sizeof(triple_mat_t));
    static triple_mat_t _B;
    memcpy(&_B, B, sizeof(triple_mat_t));
    C->_11 = _A._11 * _B._11 + _A._12 * _B._21 + _A._13 * _B._31;
    C->_12 = _A._11 * _B._12 + _A._12 * _B._22 + _A._13 * _B._32;
    C->_13 = _A._11 * _B._13 + _A._12 * _B._23 + _A._13 * _B._33;

    C->_21 = _A._21 * _B._11 + _A._22 * _B._21 + _A._23 * _B._31;
    C->_22 = _A._21 * _B._12 + _A._22 * _B._22 + _A._23 * _B._32;
    C->_23 = _A._21 * _B._13 + _A._22 * _B._23 + _A._23 * _B._33;

    C->_31 = _A._31 * _B._11 + _A._32 * _B._21 + _A._33 * _B._31;
    C->_32 = _A._31 * _B._12 + _A._32 * _B._22 + _A._33 * _B._32;
    C->_33 = _A._31 * _B._13 + _A._32 * _B._23 + _A._33 * _B._33;
}
void triple_mat_add(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C) {
    C->_11 = A->_11 + B->_11;
    C->_12 = A->_12 + B->_12;
    C->_13 = A->_13 + B->_13;

    C->_21 = A->_21 + B->_21;
    C->_22 = A->_22 + B->_22;
    C->_23 = A->_23 + B->_23;

    C->_31 = A->_31 + B->_31;
    C->_32 = A->_32 + B->_32;
    C->_33 = A->_33 + B->_33;
}
void triple_mat_sub(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C) {
    C->_11 = A->_11 - B->_11;
    C->_12 = A->_12 - B->_12;
    C->_13 = A->_13 - B->_13;

    C->_21 = A->_21 - B->_21;
    C->_22 = A->_22 - B->_22;
    C->_23 = A->_23 - B->_23;

    C->_31 = A->_31 - B->_31;
    C->_32 = A->_32 - B->_32;
    C->_33 = A->_33 - B->_33;
}
void triple_mat_transpose(triple_mat_t *A, triple_mat_t *C) {
    static triple_mat_t _A;
    memcpy(&_A, A, sizeof(triple_mat_t));
    C->_11 = _A._11; C->_21 = _A._12; C->_31 = _A._13;
    C->_12 = _A._21; C->_22 = _A._22; C->_32 = _A._23;
    C->_13 = _A._31; C->_23 = _A._32; C->_33 = _A._33;
}
bool triple_mat_inv(triple_mat_t *A, triple_mat_t *A_inv) {
    // 计算行列式
    float det = A->_11 * (A->_22 * A->_33 - A->_23 * A->_32)
              - A->_12 * (A->_21 * A->_33 - A->_23 * A->_31)
              + A->_13 * (A->_21 * A->_32 - A->_22 * A->_31);

    // 如果行列式为零，则矩阵不可逆
    if (det == 0) {
        return false; // 矩阵不可逆
    }
    static triple_mat_t _A;
    memcpy(&_A, A, sizeof(triple_mat_t));
    // 计算伴随矩阵
    A_inv->_11 = (_A._22 * _A._33 - _A._23 * _A._32) / det;
    A_inv->_12 = (_A._13 * _A._32 - _A._12 * _A._33) / det;
    A_inv->_13 = (_A._12 * _A._23 - _A._13 * _A._22) / det;
    A_inv->_21 = (_A._23 * _A._31 - _A._21 * _A._33) / det;
    A_inv->_22 = (_A._11 * _A._33 - _A._13 * _A._31) / det;
    A_inv->_23 = (_A._13 * _A._21 - _A._11 * _A._23) / det;
    A_inv->_31 = (_A._21 * _A._32 - _A._22 * _A._31) / det;
    A_inv->_32 = (_A._12 * _A._31 - _A._11 * _A._32) / det;
    A_inv->_33 = (_A._11 * _A._22 - _A._12 * _A._21) / det;

    return true; // 成功计算逆矩阵
}
