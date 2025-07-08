#ifndef MATRIX_H_
#define MATRIX_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <math.h>
#include "stdbool.h"

typedef struct {
  float _11;
  float _21;
} dual_vec_t;
typedef struct {
  float _11;
  float _12;
} dual_vec_T_t;
typedef struct {
  float _11;
  float _12;
  float _21;
  float _22;
} quad_vec_t;
typedef struct {
  float _11;
  float _21;
  float _31;
} triple_vec_t;
#pragma anon_unions
typedef union{
  struct {
    float _11;
    float _12;
    float _13;
    float _21;
    float _22;
    float _23;
    float _31;
    float _32;
    float _33;
  };
  float _[3][3];
}triple_mat_t;

#ifndef DEFINED_get_quad_vec_inv
#define DEFINED_get_quad_vec_inv
/**
 * @brief Get the quad vec inv object
 *
 * @param M the matrix that requires inversion
 * @return quad_vec_t
 * @ref https://zhuanlan.zhihu.com/p/558667088
 */
static inline quad_vec_t get_quad_vec_inv(quad_vec_t *M) {
  float det_M = M->_11 * M->_22 - M->_12 * M->_21;
  // quad_vec_t M_inv = {
  //     ._11 = M->_22 / det_M,
  //     ._22 = M->_11 / det_M,
  //     ._12 = -M->_12 / det_M,
  //     ._21 = -M->_21 / det_M,
  // };
  quad_vec_t M_inv = {
      ._11 = M->_22 / det_M,
      ._12 = -M->_12 / det_M,
      ._21 = -M->_21 / det_M,
      ._22 = M->_11 / det_M,
  };

  return M_inv;
}

void quad_matvec_mult(quad_vec_t *A, dual_vec_t *B, dual_vec_t *C);
void quad_mat_mult(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C);
void quad_mat_add(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C);
void quad_mat_sub(quad_vec_t *A, quad_vec_t *B, quad_vec_t *C);
void quad_mat_transpose(quad_vec_t *A, quad_vec_t *C);
bool quad_mat_inv(quad_vec_t *A, quad_vec_t *A_inv);

void triple_matvec_mult(triple_mat_t *A, triple_vec_t *B, triple_vec_t *C);
void triple_mat_mult(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C);
void triple_mat_add(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C);
void triple_mat_sub(triple_mat_t *A, triple_mat_t *B, triple_mat_t *C);
void triple_mat_transpose(triple_mat_t *A, triple_mat_t *C);
bool triple_mat_inv(triple_mat_t *A, triple_mat_t *A_inv);

#endif
#ifdef __cplusplus
}
#endif
#endif // !MATRIX_H_
