/*
  ==============================================================================
  
  ICP.H - WIP! 
  
  This is a header file library for producing alignment between two pointsets
  with normals.

  #define ICP_IMPLEMENTATION
  #include "icp.h"

  ==============================================================================
  DEPENDENCIES
    msh.h
    msh_vec_math.h
    msh_hash_grid.h
    
  ==============================================================================
  AUTHORS

    Maciej S. Halber (macikuh@gmail.com)
  ==============================================================================
  LICENSE

  Copyright (c) 2017 Maciej S. Halber

  Permission is hereby granted, free of charge, to any person obtaining a copy 
  of this software and associated documentation files (the "Software"), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
  copies of the Software, and to permit persons to whom the Software is 
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
  SOFTWARE.

  ==============================================================================
  TODOs:
  [ ] Const correctness
  [ ] How does simon performs icp?
  [ ] Simplified api
  [ ] Experiment with data layouts
  [ ] Provide more raw interface support(to be usable even if msh_libs are not present)
  [ ] Add robust function support
  
  ==============================================================================
  REFERENCES:
  [1] Rusinkiewicz & Levoy '01; Efficient Variants of the ICP algorithm
      http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.23.9388&rep=rep1&type=pdf
  [2] Low '04; Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration
      http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.298.4533&rep=rep1&type=pdf
  [3] Sorkine-Hornung & Rabinovich '17; Least-Squares Rigid Motion Using SVD
      https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
*/


#ifndef ICP_H
#define ICP_H


#include <algorithm>
#include <limits>
#include "lineqn.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ICP_STATIC
#define ICPDEF static
#else
#define ICPDEF extern
#endif

// This function will modify T1 so that T1*pts1 aligns to T2*pts2
ICPDEF float 
icp_align(msh_vec3_t* pts1, msh_vec3_t* nor1, int32_t n_pts1,
          msh_vec3_t* pts2, msh_vec3_t* nor2, int32_t n_pts2,
          msh_mat4_t* T1, msh_mat4_t T2, 
          float max_dist, float max_angle, bool verbose);

// function to build your own variant of icp
ICPDEF float 
icp_estimate_rigid_xform_pt2pt(msh_vec3_t* pts1, msh_vec3_t* pts2, 
                               float* weights, int32_t n_pts, 
                               msh_mat4_t* T1);
ICPDEF float 
icp_estimate_rigid_xform_pt2pl(msh_vec3_t* pts1, msh_vec3_t* pts2, 
                               msh_vec3_t* nor2, float* weights, 
                               int32_t n_pts, msh_mat4_t* T1);

// NOTE(maciej): This function signature is a bit crazy. Need to simplify with
// some structs?
ICPDEF void 
icp_find_corrs( /* Input pointsets + search indexes */
               msh_vec3_t* pts1, msh_vec3_t* nor1, int32_t n_pts1, 
               msh_hash_grid_t* idx1,
               msh_vec3_t* pts2, msh_vec3_t* nor2, int32_t n_pts2, 
               msh_hash_grid_t* idx2,
               /* Local-to-world transformations */
               msh_mat4_t T1, msh_mat4_t T2,
               /* Output correspondences */
               msh_vec3_t** corr_pts1, msh_vec3_t** corr_nor1,
               msh_vec3_t** corr_pts2, msh_vec3_t** corr_nor2,
               float** weights, int32_t* n_corrs,
               /* Parameters */
               float max_dist, float max_angle);

#ifdef __cplusplus
}
#endif

#endif /*ICP_H*/
////////////////////////////////////////////////////////////////////////////////
#ifdef ICP_IMPLEMENTATION
static msh_vec3_t
icp__compute_centroid(msh_vec3_t* pts, int32_t n_pts)
{
  msh_vec3_t centroid = msh_vec3_zeros();
  for(int i = 0; i < n_pts; ++i)
  {
    centroid = msh_vec3_add(centroid, pts[i]);
  }
  centroid = msh_vec3_scalar_div(centroid, (float)n_pts);
  return centroid;
}

static msh_vec3_t
icp__compute_weighted_centroid(msh_vec3_t* pts, float* weights, int32_t n_pts)
{
  msh_vec3_t centroid = msh_vec3_zeros();
  float total_weight = 0.0f;
  for(int i = 0; i < n_pts; ++i)
  {
    total_weight += weights[i];
    centroid = msh_vec3_add(centroid, msh_vec3_scalar_mul(pts[i], weights[i]));
  }
  centroid = msh_vec3_scalar_div(centroid, total_weight);
  return centroid;
}


// TODO(maciej): Assert that we need at least 3 pts
// NOTE(maciej): Method Reference: Besl&McKay '92
ICPDEF float 
icp_estimate_rigid_xform_pt2pt(msh_vec3_t* pts1, msh_vec3_t* pts2, 
                               float* weights, int32_t n_pts, 
                               msh_mat4_t* T1)
{
//   // Compute centroids
//   msh_vec3_t c1 = icp__compute_weighted_centroid(pts1, weights, n_pts);
//   msh_vec3_t c2 = icp__compute_weighted_centroid(pts2, weights, n_pts);

//   // Accumulate covariance and calculate current alignment error
//   double sum = 0.0;
//   double total_weight = 0.0;
//   msh_mat3_t C = msh_mat3_zeros();
//   for(int32_t i = 0; i < n_pts; ++i)
//   {
//     msh_mat3_t Ci = msh_mat3_zeros();
//     Ci = msh_vec3_outer_product(msh_vec3_sub(pts1[i], c1),
//                                 msh_vec3_sub(pts2[i], c2));
//     Ci = msh_mat3_scalar_mul(Ci, weights[i]);
//     C = msh_mat3_add(C, Ci);

//     sum += weights[i] * msh_vec3_norm_sq(msh_vec3_sub(pts1[i], pts2[i])); 
//     total_weight += weights[i];
//   }
//   float err = (float)sqrt(sum/total_weight);

//   // Compure SVD
//   Eigen::Matrix3d eC;
//   eC << C.col[0].x, C.col[1].x, C.col[2].x,
//         C.col[0].y, C.col[1].y, C.col[2].y,
//         C.col[0].z, C.col[1].z, C.col[2].z;
//   Eigen::JacobiSVD<Eigen::Matrix3d> svd( eC, Eigen::ComputeFullU |
//                                              Eigen::ComputeFullV );

//   // Extract transformation
//   Eigen::Matrix3d reg = Eigen::Matrix3d::Identity();
//   reg(2,2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
//   Eigen::Matrix3d eR = svd.matrixV() * reg * svd.matrixU().transpose();

//   msh_mat3_t R = msh_mat3_identity();
//   R.col[0] = msh_vec3((float)eR(0, 0), (float)eR(1, 0), (float)eR(2, 0)); 
//   R.col[1] = msh_vec3((float)eR(0, 1), (float)eR(1, 1), (float)eR(2, 1)); 
//   R.col[2] = msh_vec3((float)eR(0, 2), (float)eR(1, 2), (float)eR(2, 2)); 
//   msh_vec3_t t = msh_vec3_sub(c2, msh_mat3_vec3_mul(R, c1));

//   msh_mat4_t T = msh_mat3_to_mat4(R);
//   T.col[3] = msh_vec4(t.x, t.y, t.z, 1.0);
  
//   // Update the transformation
//   *T1 = msh_mat4_mul(T,*T1);// NOTE(maciej): Accumulation.. Should be done with doubles.

//   return err;
return 1.0f;
}

// Reference [Kok-Lim Low'04] 
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.298.4533&rep=rep1&type=pdf
ICPDEF float 
icp_estimate_rigid_xform_pt2pl(msh_vec3_t* pts1, msh_vec3_t* pts2, 
                               msh_vec3_t *nor2, float* weights, 
                               int32_t n_pts, msh_mat4_t* T1)
{
  // Compute centroids
  // TODO(maciej): No need to keep recomputing these.
  // TODO(maciej): These also need double based vec3 ...
  msh_vec3_t c1 = icp__compute_weighted_centroid(pts1, weights, n_pts);
  msh_vec3_t c2 = icp__compute_weighted_centroid(pts2, weights, n_pts);

  // Accumulate covariance and calculate current alignment error
  double sum = 0.0;
  double total_weight = 0.0;

  // accumulation block matrices (to build covariance later) and result vector
  msh_mat3_t TL = msh_mat3_zeros(); // Top left
  msh_mat3_t TR = msh_mat3_zeros(); // Top right
  msh_mat3_t BR = msh_mat3_zeros(); // Bottom right
  float vec_b[6] = {0};             // Right hand side
  for(int32_t i=0; i < n_pts; ++i)
  {
    msh_vec3_t p = msh_vec3_sub(pts1[i], c1); // NOTE(maciej): Should I do centering?
    msh_vec3_t q = msh_vec3_sub(pts2[i], c2);
    msh_vec3_t n = nor2[i];

    float w = weights[i];
    msh_vec3_t d = msh_vec3_sub(p, q);
    msh_vec3_t c = msh_vec3_cross(p, n);
    float dDotn = msh_vec3_dot(d, n);
    TL = msh_mat3_add(TL, msh_mat3_scalar_mul(msh_vec3_outer_product(c,c), w));
    TR = msh_mat3_add(TR, msh_mat3_scalar_mul(msh_vec3_outer_product(c,n), w));
    BR = msh_mat3_add(BR, msh_mat3_scalar_mul(msh_vec3_outer_product(n,n), w));
    vec_b[0] += w * c.x * dDotn;
    vec_b[1] += w * c.y * dDotn;
    vec_b[2] += w * c.z * dDotn;
    vec_b[3] += w * n.x * dDotn;
    vec_b[4] += w * n.y * dDotn;
    vec_b[5] += w * n.z * dDotn;

    sum += w * dDotn * dDotn;
    total_weight += w;
  }
  float err = (float)sqrt(sum/total_weight);

  // Eigen::Matrix<double, 6, 6> C;
  // Eigen::Matrix<double, 6, 1> b;
  // C << TL.col[0].x,TL.col[1].x,TL.col[2].x,TR.col[0].x,TR.col[1].x,TR.col[2].x,
  //      TL.col[0].y,TL.col[1].y,TL.col[2].y,TR.col[0].y,TR.col[1].y,TR.col[2].y,
  //      TL.col[0].z,TL.col[1].z,TL.col[2].z,TR.col[0].z,TR.col[1].z,TR.col[2].z,
  //      TR.col[0].x,TR.col[0].y,TR.col[0].z,BR.col[0].x,BR.col[1].x,BR.col[2].x,
  //      TR.col[1].x,TR.col[1].y,TR.col[1].z,BR.col[0].y,BR.col[1].y,BR.col[2].y,
  //      TR.col[2].x,TR.col[2].y,TR.col[2].z,BR.col[0].z,BR.col[1].z,BR.col[2].z;
  // b << -vec_b[0], -vec_b[1], -vec_b[2], -vec_b[3], -vec_b[4], -vec_b[5];
  // Eigen::Matrix<double, 6, 1> x = C.ldlt().solve(b);

  // NOTE(maciej): Simon points out that this might be unstable?
  double C[6][6] = { TL.col[0].x,TL.col[1].x,TL.col[2].x,TR.col[0].x,TR.col[1].x,TR.col[2].x,
                     TL.col[0].y,TL.col[1].y,TL.col[2].y,TR.col[0].y,TR.col[1].y,TR.col[2].y,
                     TL.col[0].z,TL.col[1].z,TL.col[2].z,TR.col[0].z,TR.col[1].z,TR.col[2].z,
                     TR.col[0].x,TR.col[0].y,TR.col[0].z,BR.col[0].x,BR.col[1].x,BR.col[2].x,
                     TR.col[1].x,TR.col[1].y,TR.col[1].z,BR.col[0].y,BR.col[1].y,BR.col[2].y,
                     TR.col[2].x,TR.col[2].y,TR.col[2].z,BR.col[0].z,BR.col[1].z,BR.col[2].z };
  double b[6] = { -vec_b[0], -vec_b[1], -vec_b[2], -vec_b[3], -vec_b[4], -vec_b[5] };
  double x[6] = {0};
  double rdiag[6] = {0};
  trimesh::ldltdc<double, 6>( C, rdiag );
  trimesh::ldltsl<double, 6>( C, rdiag, b, x );

  // NOTE(maciej): Again, this probably should be done with doubles
  msh_mat4_t T = msh_mat4_identity();  
  T = msh_translate(T, c1); 
  // T = msh_translate(T, msh_vec3((float)x(3), (float)x(4), (float)x(5)));
  // T = msh_rotate(T, (float)x(0), msh_vec3(1.0f, 0.0f, 0.0f));
  // T = msh_rotate(T, (float)x(1), msh_vec3(0.0f, 1.0f, 0.0f));
  // T = msh_rotate(T, (float)x(2), msh_vec3(0.0f, 0.0f, 1.0f));

  T = msh_translate(T, msh_vec3((float)x[3], (float)x[4], (float)x[5]));
  T = msh_rotate(T, (float)x[0], msh_vec3(1.0f, 0.0f, 0.0f));
  T = msh_rotate(T, (float)x[1], msh_vec3(0.0f, 1.0f, 0.0f));
  T = msh_rotate(T, (float)x[2], msh_vec3(0.0f, 0.0f, 1.0f));
  
  T = msh_translate(T, msh_vec3_invert(c1));
  
  // Update the transformation
  *T1 = msh_mat4_mul(T,*T1);

  return err;
}

// TODO(maciej): Make normals optional
// TODO(maciej): Remove the correspondences to the boundaries.
// TODO(maciej): Do a symmetric search?
// TODO(maciej): Add transformations for both pointsets.
// TODO(maciej): Do a serious benchmarking to determine optimial data layout
// TODO(maciej): Describe variables space
ICPDEF void 
icp_find_corrs(msh_vec3_t* pts1, msh_vec3_t* nor1, int32_t n_pts1, 
               msh_hash_grid_t *index1,
               msh_vec3_t* pts2, msh_vec3_t* nor2, int32_t n_pts2, 
               msh_hash_grid_t *index2,
               msh_mat4_t T1, msh_mat4_t T2,
               msh_vec3_t** corr_pts1, msh_vec3_t** corr_nor1,
               msh_vec3_t** corr_pts2, msh_vec3_t** corr_nor2,
               float** weights, int32_t* n_corrs,
               float max_dist, float max_angle)
{
  if(*corr_pts1) {free(*corr_pts1); *corr_pts1 = NULL;}
  if(*corr_pts2) {free(*corr_pts2); *corr_pts2 = NULL;}
  if(*corr_pts1) {free(*corr_pts2); *corr_pts2 = NULL;}
  if(*corr_nor2) {free(*corr_nor2); *corr_nor2 = NULL;}
  if(*weights)   {free(*weights);   *weights = NULL;}

  *corr_pts1   = (msh_vec3_t*)malloc(n_pts1*sizeof(msh_vec3_t));
  *corr_pts2   = (msh_vec3_t*)malloc(n_pts1*sizeof(msh_vec3_t));
  *corr_nor1   = (msh_vec3_t*)malloc(n_pts1*sizeof(msh_vec3_t));
  *corr_nor2   = (msh_vec3_t*)malloc(n_pts1*sizeof(msh_vec3_t));
  *weights     = (float*)malloc(n_pts1*sizeof(float));

  msh_mat4_t T2i = msh_mat4_inverse(T2);
  int max_nn = 16;
  int ic = 0;

  float* dists = (float*)malloc( n_pts1 * max_nn * sizeof(float) );
  int32_t* ind  = (int32_t*)malloc( n_pts1 * max_nn * sizeof(int32_t) );
  size_t* n_neigh = (size_t*)malloc( n_pts1 * sizeof(size_t) );
  msh_vec3_t* query_pos = (msh_vec3_t*)malloc( n_pts1 * sizeof(msh_vec3_t) );
  msh_vec3_t* query_nor = (msh_vec3_t*)malloc( n_pts1 * sizeof(msh_vec3_t) );

  for( int i1=0; i1 < n_pts1; ++i1 )
  { 
    msh_vec3_t p = pts1[i1];
    msh_vec3_t n = nor1[i1];
    p = msh_mat4_vec3_mul(T1, p, 1);
    n = msh_mat4_vec3_mul(T1, n, 0);
    query_pos[i1] = msh_mat4_vec3_mul(T2i, p, 1);
    query_nor[i1] = msh_mat4_vec3_mul(T2i, n, 0);
  }

  msh_hash_grid_search_desc_t search_opts = {0};
  search_opts.query_pts    = &query_pos[0].x;
  search_opts.n_query_pts  = n_pts1;
  search_opts.distances_sq = dists;
  search_opts.indices      = ind;
  search_opts.n_neighbors  = n_neigh;
  search_opts.radius       = max_dist;
  search_opts.max_n_neigh  = max_nn;
  search_opts.sort         = 1;

  msh_hash_grid_radius_search( index2, &search_opts );

  for( int i1 = 0; i1 < n_pts1; ++i1 )
  {
    size_t cur_nn = n_neigh[i1];
    int32_t best_i2 = -1;
    float dist = 0.0f;
    float dot = 0.0f;
    for( size_t j = 0 ; j < cur_nn; ++j )
    {
      int32_t i2 = ind[ i1 * max_nn + j ];
      msh_vec3_t n = query_nor[i1];
      msh_vec3_t m = nor2[i2];
      dot = msh_vec3_dot(m, n);
      dot = msh_max(dot, 0.0f);
      if( acosf(dot) < max_angle ) 
      { 
        best_i2 = i2;
        dist = dists[ i1 * max_nn + j ];
        break;
      }
    }
    if( best_i2 != -1) 
    {
      (*corr_pts1)[ic] = query_pos[i1];
      (*corr_nor1)[ic] = query_nor[i1];
      (*corr_pts2)[ic] = pts2[best_i2];
      (*corr_nor2)[ic] = nor2[best_i2];
      (*weights)[ic]   = (1.0f - dist / max_dist) * dot;
      dists[ic] = dist;
      ic++;
    }
  }

  // Zero out weights of corrs whose distance is greater than 2.5 sigma
  float mean_dist = msh_compute_mean(dists, ic);
  float stddev_dist = msh_compute_stddev(mean_dist, dists, ic);
  if( stddev_dist > 0.000001 )
  {
    for( int i = 0 ; i < ic ; ++i )
    {
      if( dists[i] > 2.5f * stddev_dist) (*weights)[i] = 0.0;
    }
  }
  *n_corrs = ic;

  free(query_pos);
  free(query_nor);
  free(n_neigh);
  free(dists);
  free(ind);

  #undef MAX_NN
}

// TODO(maciej): Make normals optional
// TODO(maciej): Create proper angle limiting instead of the dot product
ICPDEF float 
icp_align(msh_vec3_t* pts1, msh_vec3_t* nor1, int32_t n_pts1,
          msh_vec3_t* pts2, msh_vec3_t* nor2, int32_t n_pts2,
          msh_mat4_t* T1, msh_mat4_t T2, 
          float max_dist, float max_angle, bool verbose)
{
  uint64_t gt1 = msh_time_now();
  // Correspondence storage 
  // TODO(maciej): Probably can malloc this only once instead each iteration
  msh_vec3_t* corr_pts1 = NULL;
  msh_vec3_t* corr_nor1 = NULL;
  msh_vec3_t* corr_pts2 = NULL;
  msh_vec3_t* corr_nor2 = NULL;
  float*   corr_weights = NULL;
  int32_t n_corrs = 0;

  // Search indices
  uint64_t it1 = msh_time_now();
  msh_hash_grid_t index1 = {0};
  msh_hash_grid_t index2 = {0};
  msh_hash_grid_init_3d( &index1, (float*)pts1, n_pts1, max_dist );
  msh_hash_grid_init_3d( &index2, (float*)pts2, n_pts2, max_dist );

  uint64_t it2 = msh_time_now();
  if(verbose) printf(" ICP: Search indexes build time: %fms\n", msh_time_diff_ms( it2, it1));
  float prev_err = 1e6;
  float err = 1e6;
  int max_iter = 100;
  for( int i = 0 ; i < max_iter ; ++i )
  {
    prev_err = err;
    // Perform matching and estimation steps
    uint64_t ft1 = msh_time_now();
    icp_find_corrs(pts1, nor1, n_pts1, &index1, pts2, nor2, n_pts2, &index2,
                   *T1, T2, &corr_pts1, &corr_nor1, &corr_pts2, &corr_nor2, 
                   &corr_weights, &n_corrs, max_dist, max_angle);
    uint64_t ft2 = msh_time_now();

    // Check corrs 
    if(n_corrs == 0) 
    {
      if(verbose) printf("ICP: Error: No correspondences found!\n");
      break;
    }
    
    float total_weight = 0.0;
    for( int j = 0 ; j < n_corrs; ++j )
    {
      total_weight += corr_weights[j];
    }
    if(total_weight <= 1e-7) 
    {
      if(verbose) printf("ICP: Error: All found correspondences invalid!\n");
      break;
    }

    uint64_t et1 = msh_time_now();
    if(i < 0) /* NOTE(maciej): Value greater than 0 seems to encourage sliding. */ 
      err = icp_estimate_rigid_xform_pt2pt(corr_pts1, corr_pts2, 
                                           corr_weights, n_corrs, T1);
    else
      err = icp_estimate_rigid_xform_pt2pl(corr_pts1, corr_pts2, corr_nor2, 
                                           corr_weights, n_corrs, T1);
    uint64_t et2 = msh_time_now();

    float delta = fabsf(prev_err - err);
    if(verbose) printf(" ICP: Iter %3d {Error: %7.5f; Err.Delta: %7.5f; "
           "Params: (%5.4f, %5.4f); Times: (%5.3fms, %5.3fms)}\n", 
           i, err, delta, max_dist, max_angle, 
           msh_time_diff_ms( ft2, ft1), 
           msh_time_diff_ms( et2, et1));
  
    // Stopping criterion
    if( i > 5 && delta < 1e-5 ) break;
    
    // Parameter update
    // NOTE(maciej): Need to be able to pass that in
    max_dist = msh_max( max_dist * 0.95, 0.05 );
  }
  msh_hash_grid_term( &index1 );
  msh_hash_grid_term( &index2 );
  uint64_t gt2 = msh_time_now();
  if(verbose) printf(" ICP: Full time to estimate transform: %fms\n", msh_time_diff_ms( gt2, gt1));
  return err;
}


// #include "eig3.h"
// // Estimates rigid body transformation (SE(3)) 
// // NOTE(maciej): This looks a whole lot like pca.
// static msh_mat4_t
// rsdu__estimate_se3( rs_pointcloud_t *shape, int lvl )
// {
//   msh_vec3_t centroid = msh_vec3_zeros();
//   for( int j = 0 ; j < shape->n_pts[lvl]; j++ )
//   {
//     msh_vec3_t p = shape->positions[lvl][j];
//     centroid += p;
//   }
//   centroid = msh_vec3_scalar_div( centroid, (float)shape->n_pts[lvl] );

//   msh_mat3_t covariance = msh_mat3_zeros();
//   for( int j = 0; j < shape->n_pts[lvl]; ++j )
//   {
//     msh_vec3_t p = shape->positions[lvl][j];
//     msh_vec3_t d = msh_vec3_sub(p, centroid);
//     msh_mat3_t m = msh_vec3_outer_product(d, d);
//     covariance = msh_mat3_add( covariance, m );
//   }
//   covariance = msh_mat3_scalar_div(covariance, (float)shape->n_pts[lvl]);

//   // TODO(maciej): Pull that into msh
//   double A[3][3];
//   double evec[3][3];
//   double eval[3];
//   for(int a = 0 ; a < 3; ++a)
//     for(int b = 0 ; b < 3; ++b)
//       A[a][b] = (double)covariance.col[a].data[b];
//   eig3_decomp(A, evec, eval);
//   msh_mat3_t rot = msh_mat3_identity();
//   rot.col[0] = msh_vec3((float)evec[0][0], (float)evec[1][0], (float)evec[2][0]);
//   rot.col[1] = msh_vec3((float)evec[0][1], (float)evec[1][1], (float)evec[2][1]);
//   rot.col[2] = msh_vec3((float)evec[0][2], (float)evec[1][2], (float)evec[2][2]);

//   msh_vec3_t n = msh_vec3_posy();
//   msh_vec3_t v = msh_vec3_cross(n, rot.col[2]);
//   msh_vec3_t u = msh_vec3_cross(v, n);
//   rot.col[0] = v;
//   rot.col[1] = n;
//   rot.col[2] = u;

//   msh_mat4_t xform = msh_mat3_to_mat4( rot );
//   xform.col[3] = msh_vec3_to_vec4( centroid );
//   xform.col[3].w = 1.0f;
//   return xform;
// }

#endif /*ICP_IMPLEMENTATION*/