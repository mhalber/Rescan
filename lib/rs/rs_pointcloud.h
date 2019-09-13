/*
  ==============================================================================

  RS_POINTCLOUD.H - WIP!

  This file describes the pointcloud format for rescan project files.

  #define RS_POINTCLOUD_IMPLEMENTATION
  #include "rs_pointcloud.h"

  ==============================================================================
  DEPENDENCIES
  msh.h, msh_vec_math.h, msh_ply.h, flann, hastable.h

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
  REFERENCES:
*/

#ifndef RS_POINTCLOUD_H
#define RS_POINTCLOUD_H

#ifdef __cplusplus
extern "C" {
#endif

// Multi-level pointcloud representation
#define RSPC_N_LEVELS 5

#ifdef RS_POINTCLOUD_INCLUDE_HEADERS
#include "msh/msh_std.h"
#include "msh/msh_ply.h"
#include "msh/msh_vec_math.h"
#endif

#include "cJSON.h"

// For AOS
typedef struct rs_surfel
{
  msh_vec3_t pos;
  msh_vec3_t normal;
  msh_vec3_t color;
  float radius;
  float quality;
  int32_t class_id;
  int32_t instance_id;
} rs_surfel_t;

// For SOA
typedef struct rs_pointcloud
{
  msh_vec3_t    *positions[RSPC_N_LEVELS];
  msh_vec3_t    *normals[RSPC_N_LEVELS];
  msh_vec3_t    *colors[RSPC_N_LEVELS];
  float         *radii[RSPC_N_LEVELS];
  float         *qualities[RSPC_N_LEVELS];
  int32_t       *class_ids[RSPC_N_LEVELS];
  int32_t       *instance_ids[RSPC_N_LEVELS];

  msh_hash_grid_t *search_grids[RSPC_N_LEVELS];

  size_t         n_pts[RSPC_N_LEVELS];
  msh_bbox_t     bbox;
  msh_vec3_t     centroid;
  msh_mat3_t     covariance;
  float          voxel_size[RSPC_N_LEVELS];

  int32_t        *faces_ind; // Well not really a point cloud :D
  size_t         n_faces;
} rs_pointcloud_t;

typedef enum rs_point_id
{
  RS_PT_CLASS_ID = 0,
  RS_PT_INSTANCE_ID,
  RS_PT_N_IDS
} rs_point_id_t;

rs_pointcloud_t* rs_pointcloud_init( int32_t n );
rs_pointcloud_t* rs_pointcloud_copy( const rs_pointcloud_t* pc );
rs_pointcloud_t* rs_pointcloud_copy_by_ids( const rs_pointcloud_t* pc, int32_t lvl, rs_point_id_t type, 
                                            int32_t* ids, int32_t n_ids, int32_t compute_levels );
rs_pointcloud_t* rs_pointcloud_copy_by_indices( const rs_pointcloud_t* pc, int32_t lvl, int32_t* inds, int32_t n_inds );
rs_pointcloud_t* rs_pointcloud_remove_by_ids( const rs_pointcloud_t* pc, rs_point_id_t type, 
                                              int32_t* ids, int32_t n_ids );

rs_pointcloud_t* rs_pointcloud_merge( const rs_pointcloud_t* pc_a, const rs_pointcloud_t* pc_b, int32_t lvl );

void             rs_pointcloud_compute_levels( rs_pointcloud_t* pc );
void             rs_pointcloud_compute_search_grid( rs_pointcloud_t* pc, int32_t level );
msh_vec3_t       rs_pointcloud_centroid( rs_pointcloud_t* pc, int32_t lvl );
msh_mat3_t       rs_pointcloud_covariance( rs_pointcloud_t* pc, int32_t lvl );
void             rs_pointcloud_translate( rs_pointcloud_t* pc, msh_vec3_t t, int32_t compute_levels );
void             rs_pointcloud_transform( rs_pointcloud_t* pc, msh_mat4_t t, int32_t compute_levels );
int32_t          rs_pointcloud_from_file( rs_pointcloud_t* pc, const char* filename );
int32_t          rs_pointcloud_from_files( rs_pointcloud_t* pc, const char** filenames, int32_t n,
                                           int32_t verbose_lvl );
void             rs_pointcloud_to_file( rs_pointcloud_t* pc, const char* filename, int32_t lvl = 0 );
void             rs_pointcloud_free( rs_pointcloud_t* pc, int32_t n );


#ifdef __cplusplus
}
#endif

#endif /*RS_POINTCLOUD_H*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef RS_POINTCLOUD_IMPLEMENTATION

void rs_pointcloud__compute_bbox(rs_pointcloud_t* pc);
void rs_pointcloud__compute_level( rs_pointcloud_t* pc, int32_t level );

rs_pointcloud_t*
rs_pointcloud_init( int32_t n )
{
  float resolutions[RSPC_N_LEVELS] = { 0.005f, 0.01f, 0.02f, 0.04f, 0.08f };
  rs_pointcloud_t* pcs = (rs_pointcloud_t*)malloc( n * sizeof(rs_pointcloud_t) );

  for( int32_t i = 0 ; i < n ; ++i )
  {
    memcpy( pcs[i].voxel_size, resolutions, sizeof(resolutions) );
    for( int32_t j = 0; j < RSPC_N_LEVELS; ++j )
    {
      pcs[i].positions[j]     = NULL;
      pcs[i].normals[j]       = NULL;
      pcs[i].colors[j]        = NULL;
      pcs[i].radii[j]         = NULL;
      pcs[i].qualities[j]     = NULL;
      pcs[i].class_ids[j]     = NULL;
      pcs[i].instance_ids[j]  = NULL;
      pcs[i].search_grids[j]  = NULL;
      pcs[i].n_pts[j]         = 0;
    }
    pcs[i].n_faces            = 0;
    pcs[i].faces_ind          = NULL;
    pcs[i].centroid           = msh_vec3(-MSH_F32_MAX, -MSH_F32_MAX, -MSH_F32_MAX);
    pcs[i].covariance         = msh_mat3_identity();
    pcs[i].covariance.data[0] = -MSH_F32_MAX;
  }

  return pcs;
}


rs_pointcloud_t*
rs_pointcloud_copy( const rs_pointcloud_t* in_pc )
{
  int32_t n_pts = in_pc->n_pts[0];
  rs_pointcloud_t* out_pc = rs_pointcloud_init( 1 );
  out_pc->positions[0]    = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->normals[0]      = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->colors[0]       = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->radii[0]        = (float*)malloc(n_pts*sizeof(float));
  out_pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
  out_pc->class_ids[0]    = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->n_pts[0]        = n_pts;

  for( int32_t i = 0; i < n_pts; ++i )
  {
    out_pc->positions[0][i]    = in_pc->positions[0][i];
    out_pc->normals[0][i]      = in_pc->normals[0][i];
    out_pc->colors[0][i]       = in_pc->colors[0][i];
    out_pc->radii[0][i]        = in_pc->radii[0][i];
    out_pc->qualities[0][i]    = in_pc->qualities[0][i];
    out_pc->class_ids[0][i]    = in_pc->class_ids[0][i];
    out_pc->instance_ids[0][i] = in_pc->instance_ids[0][i];
  }

  rs_pointcloud_compute_levels( out_pc );
  return out_pc;
}

rs_pointcloud_t*
rs_pointcloud_copy_by_indices( const rs_pointcloud_t* in_pc, int32_t lvl, int32_t* indices, int32_t n_inds )
{
  rs_pointcloud_t* out_pc = NULL;
  int32_t n_pts = n_inds;

  out_pc = rs_pointcloud_init( 1 );
  out_pc->positions[0]    = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->normals[0]      = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->colors[0]       = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->radii[0]        = (float*)malloc(n_pts*sizeof(float));
  out_pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
  out_pc->class_ids[0]    = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->n_pts[0] = n_pts;

  for( int32_t i = 0; i < RSPC_N_LEVELS; ++i)
  {
    out_pc->voxel_size[i] = in_pc->voxel_size[i];
  }

  for( int32_t i = 0; i < n_inds; ++i )
  {
      out_pc->positions[0][i]    = in_pc->positions[0][indices[i]];
      out_pc->normals[0][i]      = in_pc->normals[0][indices[i]];
      out_pc->colors[0][i]       = in_pc->colors[0][indices[i]];
      out_pc->radii[0][i]        = in_pc->radii[0][indices[i]];
      out_pc->qualities[0][i]    = in_pc->qualities[0][indices[i]];
      out_pc->class_ids[0][i]    = in_pc->class_ids[0][indices[i]];
      out_pc->instance_ids[0][i] = in_pc->instance_ids[0][indices[i]];
  }

  rs_pointcloud_compute_levels( out_pc );
  return out_pc;
}

rs_pointcloud_t*
rs_pointcloud_copy_by_ids( const rs_pointcloud_t* in_pc, int32_t lvl, rs_point_id_t type, int32_t* ids, int32_t n_ids, int32_t compute_levels )
{
  rs_pointcloud_t* out_pc = NULL;
  size_t n_pts = 0;
  int32_t* pc_ids = NULL;
  if( type == RS_PT_CLASS_ID ) { pc_ids = in_pc->class_ids[lvl]; }
  if( type == RS_PT_INSTANCE_ID ) { pc_ids = in_pc->instance_ids[lvl]; }
  for( size_t i = 0; i < in_pc->n_pts[lvl]; ++i )
  {
    for( size_t j = 0; j < (size_t)n_ids; ++j )
    {
      if( pc_ids[i] == ids[j] ) { n_pts++; }
    }
  }

  if( n_pts <= 0 ) return out_pc;

  out_pc = rs_pointcloud_init( 1 );
  out_pc->positions[0]    = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->normals[0]      = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->colors[0]       = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->radii[0]        = (float*)malloc(n_pts*sizeof(float));
  out_pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
  out_pc->class_ids[0]    = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->n_pts[0] = n_pts;

  for( int32_t i = 0; i < RSPC_N_LEVELS; ++i)
  {
    out_pc->voxel_size[i] = in_pc->voxel_size[i];
  }

  size_t out_idx = 0;
  for( size_t i = 0; i < in_pc->n_pts[lvl]; ++i )
  {
    for( size_t j = 0; j < (size_t)n_ids; ++j )
    {
      if( pc_ids[i] == ids[j] )
      {
        out_pc->positions[0][out_idx]    = in_pc->positions[lvl][i];
        out_pc->normals[0][out_idx]      = in_pc->normals[lvl][i];
        out_pc->colors[0][out_idx]       = in_pc->colors[lvl][i];
        out_pc->radii[0][out_idx]        = in_pc->radii[lvl][i];
        out_pc->qualities[0][out_idx]    = in_pc->qualities[lvl][i];
        out_pc->class_ids[0][out_idx]    = in_pc->class_ids[lvl][i];
        out_pc->instance_ids[0][out_idx] = in_pc->instance_ids[lvl][i];
        out_idx++;
      }
    }
  }

  rs_pointcloud__compute_bbox( out_pc );
  out_pc->centroid = msh_vec3( -MSH_F32_MAX, -MSH_F32_MAX, -MSH_F32_MAX);
  out_pc->covariance.data[0] = -MSH_F32_MAX;

  if( compute_levels ) rs_pointcloud_compute_levels( out_pc );
  return out_pc;
}

rs_pointcloud_t*
rs_pointcloud_remove_by_ids( const rs_pointcloud_t* in_pc, rs_point_id_t type, int32_t* ids, int32_t n_ids )
{
  rs_pointcloud_t* out_pc = NULL;
  size_t n_pts = 0;
  int32_t* pc_ids = NULL;
  if( type == RS_PT_CLASS_ID ) { pc_ids = in_pc->class_ids[0]; }
  if( type == RS_PT_INSTANCE_ID ) { pc_ids = in_pc->instance_ids[0]; }
  for( size_t i = 0; i < in_pc->n_pts[0]; ++i )
  {
    int32_t skip = 0;
    for( size_t j = 0; j < (size_t)n_ids; ++j )
    {
      if( pc_ids[i] == ids[j] ) { skip = 1; break; }
    }
    if( !skip ) { n_pts++; }
  }

  if( n_pts <= 0 ) return out_pc;

  out_pc = rs_pointcloud_init( 1 );
  out_pc->positions[0]    = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->normals[0]      = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->colors[0]       = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->radii[0]        = (float*)malloc(n_pts*sizeof(float));
  out_pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
  out_pc->class_ids[0]    = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->n_pts[0] = n_pts;

  for( int32_t i = 0; i < RSPC_N_LEVELS; ++i)
  {
    out_pc->voxel_size[i] = in_pc->voxel_size[i];
  }

  size_t out_idx = 0;
  for( size_t i = 0; i < in_pc->n_pts[0]; ++i )
  {
    int32_t skip = 0;
    for( size_t j = 0; j < (size_t)n_ids; ++j )
    {
      if( pc_ids[i] == ids[j] ) { skip = 1; break; }
    }
    if( !skip )
    {
      out_pc->positions[0][out_idx]    = in_pc->positions[0][i];
      out_pc->normals[0][out_idx]      = in_pc->normals[0][i];
      out_pc->colors[0][out_idx]       = in_pc->colors[0][i];
      out_pc->radii[0][out_idx]        = in_pc->radii[0][i];
      out_pc->qualities[0][out_idx]    = in_pc->qualities[0][i];
      out_pc->class_ids[0][out_idx]    = in_pc->class_ids[0][i];
      out_pc->instance_ids[0][out_idx] = in_pc->instance_ids[0][i];
      out_idx++;
    }
  }

  rs_pointcloud_compute_levels( out_pc );
  return out_pc;
}

void
rs_pointcloud_swap_vec3( msh_vec3_t* a, msh_vec3_t* b )
{
  msh_vec3_t tmp = *a;
  *a = *b;
  *b = tmp;
}

void
rs_pointcloud_swap_int32( int32_t* a, int32_t* b )
{
  int32_t tmp = *a;
  *a = *b;
  *b = tmp;
}

void
rs_pointcloud_swap_float( float* a, float* b )
{
  float tmp = *a;
  *a = *b;
  *b = tmp;
}

rs_pointcloud_t*
rs_pointcloud_merge( const rs_pointcloud_t* pc_a, const rs_pointcloud_t* pc_b, int32_t lvl)
{
  rs_pointcloud_t* out_pc = NULL;
  size_t n_pts = pc_a->n_pts[lvl] + pc_b->n_pts[lvl];

  if( n_pts <= 0 ) { return out_pc; }

  out_pc = rs_pointcloud_init( 1 );
  out_pc->positions[0]    = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->normals[0]      = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->colors[0]       = (msh_vec3_t*)malloc(n_pts*sizeof(msh_vec3_t));
  out_pc->radii[0]        = (float*)malloc(n_pts*sizeof(float));
  out_pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
  out_pc->class_ids[0]    = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
  out_pc->n_pts[0] = n_pts;
  size_t out_idx = 0;

  for( size_t i = 0; i < pc_a->n_pts[lvl]; ++i )
  {
    out_pc->positions[0][out_idx]    = pc_a->positions[lvl][i];
    out_pc->normals[0][out_idx]      = pc_a->normals[lvl][i];
    out_pc->colors[0][out_idx]       = pc_a->colors[lvl][i];
    out_pc->radii[0][out_idx]        = pc_a->radii[lvl][i];
    out_pc->qualities[0][out_idx]    = pc_a->qualities[lvl][i];
    out_pc->class_ids[0][out_idx]    = pc_a->class_ids[lvl][i];
    out_pc->instance_ids[0][out_idx] = pc_a->instance_ids[lvl][i];
    out_idx++;
  }

  for( size_t i = 0; i < pc_b->n_pts[lvl]; ++i )
  {
    out_pc->positions[0][out_idx]    = pc_b->positions[lvl][i];
    out_pc->normals[0][out_idx]      = pc_b->normals[lvl][i];
    out_pc->colors[0][out_idx]       = pc_b->colors[lvl][i];
    out_pc->radii[0][out_idx]        = pc_b->radii[lvl][i];
    out_pc->qualities[0][out_idx]    = pc_b->qualities[lvl][i];
    out_pc->class_ids[0][out_idx]    = pc_b->class_ids[lvl][i];
    out_pc->instance_ids[0][out_idx] = pc_b->instance_ids[lvl][i];
    out_idx++;
  }


  msh_rand_ctx_t rand_gen = {0};
  msh_rand_init( &rand_gen, 12346ULL );

  for( int32_t i = n_pts - 1; i > 0; i-- ) 
  { 
    int32_t j = msh_rand_nextf( &rand_gen ) * i;
    // Swap arr[i] with the element at random index 
    rs_pointcloud_swap_vec3( &out_pc->positions[0][i],  &out_pc->positions[0][j] );
    rs_pointcloud_swap_vec3( &out_pc->normals[0][i],  &out_pc->normals[0][j] );
    rs_pointcloud_swap_vec3( &out_pc->colors[0][i],  &out_pc->colors[0][j] );
    rs_pointcloud_swap_float( &out_pc->radii[0][i],  &out_pc->radii[0][j] );
    rs_pointcloud_swap_float( &out_pc->qualities[0][i],  &out_pc->qualities[0][j] );
    rs_pointcloud_swap_int32( &out_pc->class_ids[0][i],  &out_pc->class_ids[0][j] );
    rs_pointcloud_swap_int32( &out_pc->instance_ids[0][i],  &out_pc->instance_ids[0][j] );

  } 

  rs_pointcloud_compute_levels( out_pc );
  return out_pc;
}

inline float
rs__exp_score_sq( float dist_sq, float sigma )
{
  return expf( -dist_sq / (2.0f*sigma*sigma) );
}

float
rs_pointcloud__pt2pt_alignment_score( rs_pointcloud_t* pc_a,
                                      rs_pointcloud_t* pc_b,
                                      msh_mat4_t xform,
                                      float dist_threshold,
                                      int32_t lvl )
{
  int32_t n_pts_a        = pc_a->n_pts[lvl];
  int32_t n_pts_b        = pc_b->n_pts[lvl];
  float radius       = dist_threshold;
  float factor       = 0.1f;
  float score        = 0.0f;

  // a2b
  msh_vec3_t* positions = (msh_vec3_t*)malloc( n_pts_a * sizeof(msh_vec3_t) );
  size_t* n_neighbors   = (size_t*)malloc( n_pts_a * sizeof( size_t ) );
  int32_t* indices          = (int32_t*)malloc( n_pts_a * sizeof(int32_t) );
  float* dists          = (float*)malloc( n_pts_a * sizeof(float) );

  for( int32_t i = 0; i < n_pts_a; ++i ) { positions[i] = msh_mat4_vec3_mul(xform,
                                                        pc_a->positions[lvl][i], 1); }


  msh_hash_grid_search_desc_t search_opts = {0};
  search_opts.query_pts = &positions[0].x;
  search_opts.n_query_pts = n_pts_a;
  search_opts.distances_sq = dists;
  search_opts.indices = indices;
  search_opts.n_neighbors = n_neighbors;
  search_opts.radius = radius;
  search_opts.max_n_neigh = 1;
  msh_hash_grid_radius_search( pc_b->search_grids[lvl], &search_opts );

  for( int32_t i = 0; i < n_pts_a; i++ )
  {
    if( !n_neighbors[i] ) { continue; }
    score += rs__exp_score_sq( dists[i], factor );
  }

  free( indices );
  free( dists );
  free( n_neighbors );
  free( positions );

  // b2a
  positions = (msh_vec3_t*)malloc( n_pts_b*sizeof(msh_vec3_t) );
  n_neighbors   = (size_t*)malloc( n_pts_a * sizeof( size_t ) );
  indices   = (int32_t*)malloc( n_pts_b*sizeof(int32_t) );
  dists     = (float*)malloc( n_pts_b*sizeof(float) );
  msh_mat4_t inv_xform = msh_mat4_inverse(xform);

  for( int32_t i = 0; i < n_pts_b; ++i ) { positions[i] = msh_mat4_vec3_mul(inv_xform,
                                                        pc_b->positions[lvl][i], 1); }
  search_opts.query_pts = &positions[0].x;
  search_opts.n_query_pts = n_pts_a;
  search_opts.distances_sq = dists;
  search_opts.indices = indices;
  search_opts.n_neighbors = n_neighbors;
  search_opts.radius = radius;
  search_opts.max_n_neigh = 1;
  msh_hash_grid_radius_search( pc_a->search_grids[lvl], &search_opts );

  
  for( int32_t i = 0; i < n_pts_b; i++ )
  {
    if( !n_neighbors[i] ) { continue; }
    score += rs__exp_score_sq( dists[i], factor );
  }

  free( indices );
  free( dists );
  free( n_neighbors );
  free( positions );

  score /= (n_pts_a + n_pts_b);
  return score;
}


void
rs_pointcloud_free( rs_pointcloud_t* pcs, int32_t n )
{
  for(int32_t i = 0 ; i < n ; ++i)
  {

    for( int32_t j = 0; j < RSPC_N_LEVELS; ++j )
    {
      if( pcs[i].search_grids[j] ) { msh_hash_grid_term( pcs[i].search_grids[j] ); }
      free( pcs[i].positions[j] );
      free( pcs[i].normals[j] );
      free( pcs[i].colors[j] );
      free( pcs[i].radii[j] );
      free( pcs[i].qualities[j] );
      free( pcs[i].class_ids[j] );
      free( pcs[i].instance_ids[j] );
      free( pcs[i].search_grids[j] );
      pcs[i].n_pts[j] = 0;
    }
    free( pcs[i].faces_ind );
  }
}

int32_t
rs_pointcloud__compute_normals(rs_pointcloud_t* pc, int32_t* faces_ind, int32_t faces_ind_count )
{
  int32_t* counts = (int32_t*)malloc(pc->n_pts[0] * sizeof(int32_t));
  memset(counts, 0, pc->n_pts[0] * sizeof(int32_t));

  // Compute face normals and smooth into vertex normals
  for( size_t i = 0; i < pc->n_pts[0 ]; i++ )
  {
    pc->normals[0][i] = msh_vec3_zeros();
  }

  for( size_t i = 0; i < (size_t)faces_ind_count; i++ )
  {
    const int32_t fbase = 3*i;
    const uint32_t i1 = faces_ind[fbase];
    const uint32_t i2 = faces_ind[fbase+1];
    const uint32_t i3 = faces_ind[fbase+2];
    msh_vec3_t p1 = pc->positions[0][i1];
    msh_vec3_t p2 = pc->positions[0][i2];
    msh_vec3_t p3 = pc->positions[0][i3];

    // smoothly blend face normals into vertex normals
    msh_vec3_t normal  = msh_vec3_cross(msh_vec3_sub(p2, p1), msh_vec3_sub(p3, p1));
    pc->normals[0][i1] = msh_vec3_lerp( pc->normals[0][i1], normal, 1.0f / (counts[i1] + 1.0f));
    pc->normals[0][i2] = msh_vec3_lerp( pc->normals[0][i2], normal, 1.0f / (counts[i2] + 1.0f));
    pc->normals[0][i3] = msh_vec3_lerp( pc->normals[0][i3], normal, 1.0f / (counts[i3] + 1.0f));
    counts[i1]++; counts[i2]++; counts[i3]++;
  }

  for( size_t i = 0; i < pc->n_pts[0]; i++ )
  {
    float norm = msh_vec3_norm( pc->normals[0][i] );
    if( norm > 0.0 ) pc->normals[0][i] = msh_vec3_normalize(pc->normals[0][i]);
    else             pc->normals[0][i] = msh_vec3_posy();

  }

  free(counts);
  return 0;
}

int32_t
rs_pointcloud__load_ply(const char* filename, rs_pointcloud_t* pc, int32_t level)
{
  msh_ply_t* pf = msh_ply_open( filename, "rb");
  int32_t error = MSH_PLY_FILE_OPEN_ERR;
  if( pf )
  {
    error = msh_ply_parse_header( pf );
    if( error ) { printf("Could not parse header of the ply file!\n" ); }

     const char* vertex_attributes[] = { "x", "y", "z",
                                         "nx", "ny", "nz",
                                         "red", "green", "blue",
                                         "radius", "class_idx", "instance_idx" };
    size_t posns_count, norms_count, colrs_count,
           radii_count, class_count, instc_count, faces_ind_count;
    posns_count = norms_count = colrs_count =
    radii_count = class_count = instc_count = faces_ind_count = 0;

    uint8_t* colors_tmp = NULL;

    msh_ply_desc_t base_vertex_desc = { .element_name = (char*)"vertex",
                                        .property_names = &vertex_attributes[0],
                                        .num_properties = 3,
                                        .data_type = MSH_PLY_FLOAT,
                                        .list_type = MSH_PLY_INVALID,
                                        .data = NULL,
                                        .list_data = NULL,
                                        .data_count = NULL,
                                        .list_size_hint = 0 };

    msh_ply_element_t* vertex_specification = msh_ply_find_element( pf, "vertex" );
    msh_ply_element_t* face_specification   = msh_ply_find_element( pf, "face" );
    msh_ply_desc_t posns_desc, norms_desc, colrs_desc, radii_desc, class_desc, instc_desc;
    posns_desc = norms_desc = colrs_desc = radii_desc = class_desc = instc_desc = base_vertex_desc;
    if( !vertex_specification )
    {
      printf("Ply file does not have \"vertex\" element!\n"); return 0;
    }

    if( msh_ply_find_property(vertex_specification, vertex_attributes[0]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[1]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[2]) )
    {
      posns_desc.data = &pc->positions[0];
      posns_desc.data_count = &posns_count;
      msh_ply_add_descriptor( pf, &posns_desc );
    }

    if( msh_ply_find_property(vertex_specification, vertex_attributes[3]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[4]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[5]) )
    {
      norms_desc.property_names = &vertex_attributes[3],
      norms_desc.data = &pc->normals[0];
      norms_desc.data_count = &norms_count;
      msh_ply_add_descriptor( pf, &norms_desc );
    }

    if( msh_ply_find_property(vertex_specification, vertex_attributes[6]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[7]) &&
        msh_ply_find_property(vertex_specification, vertex_attributes[8]) )
    {
      colrs_desc.property_names = &vertex_attributes[6],
      colrs_desc.data = &colors_tmp;
      colrs_desc.data_type = MSH_PLY_UINT8;
      colrs_desc.data_count = &colrs_count;
      msh_ply_add_descriptor( pf, &colrs_desc );
    }
    if( msh_ply_find_property(vertex_specification, vertex_attributes[9]) )
    {
      radii_desc.property_names = &vertex_attributes[9],
      radii_desc.num_properties = 1;
      radii_desc.data = &pc->radii[0];
      radii_desc.data_count = &radii_count;
      msh_ply_add_descriptor( pf, &radii_desc );
    }

    if( msh_ply_find_property(vertex_specification, vertex_attributes[10]) )
    {
      class_desc.property_names = &vertex_attributes[10],
      class_desc.num_properties = 1;
      class_desc.data = &pc->class_ids[0];
      class_desc.data_type = MSH_PLY_INT32;
      class_desc.data_count = &class_count;
      msh_ply_add_descriptor( pf, &class_desc );
    }

    if( msh_ply_find_property(vertex_specification, vertex_attributes[11]) )
    {
      instc_desc.property_names = &vertex_attributes[11],
      instc_desc.num_properties = 1;
      instc_desc.data = &pc->instance_ids[0];
      instc_desc.data_type = MSH_PLY_INT32;
      instc_desc.data_count = &instc_count;
      msh_ply_add_descriptor( pf, &instc_desc );
    }

    if( face_specification )
    {
      const char* face_attributes[] = { "vertex_indices" };
      msh_ply_desc_t base_face_desc = { .element_name = (char*)"face",
                                        .property_names = &face_attributes[0],
                                        .num_properties = 1,
                                        .data_type = MSH_PLY_INT32,
                                        .list_type = MSH_PLY_UINT32,
                                        .data = &pc->faces_ind,
                                        .list_data = NULL,
                                        .data_count = &pc->n_faces,
                                        .list_size_hint = 3 };
      msh_ply_add_descriptor( pf, &base_face_desc );
    }

    error = msh_ply_read(pf);
    msh_cprintf(error, "%s\n", msh_ply_error_msg( error ) );
    int32_t n_pts = posns_count;
    pc->n_pts[0] = n_pts;

    if( !norms_count )
    {
      pc->normals[0] = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t) );
      if(pc->n_faces)
      {
        rs_pointcloud__compute_normals( pc, pc->faces_ind, pc->n_faces );
      }
    }

    if( !radii_count )
    {
      pc->radii[0] = (float*)malloc( n_pts * sizeof(float));
      for( int32_t i = 0; i < n_pts; ++i ) { pc->radii[0][i] = 0.01; }
    }

    if( !class_count )
    {
      pc->class_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
      memset( pc->class_ids[0], 0, n_pts*sizeof(int32_t) );
    }

    if( !instc_count )
    {
      pc->instance_ids[0] = (int32_t*)malloc(n_pts*sizeof(int32_t));
      memset( pc->instance_ids[0], 0, n_pts*sizeof(int32_t) );
    }

    // normalize normals
    int32_t found_nan = 0;
    for( int32_t i = 0; i < n_pts; ++i)
    {
      pc->normals[0][i] = msh_vec3_normalize(pc->normals[0][i]);
      if( msh_isnan(pc->normals[0][i].x) ||
          msh_isnan(pc->normals[0][i].y) ||
          msh_isnan(pc->normals[0][i].z) ) { found_nan++; pc->normals[0][i]=msh_vec3_zeros();}
    }
    msh_cprintf( found_nan, "WARNING: Found %d nans in normals\n", found_nan);

    // colors need some conversion
    if( !colrs_count )
    {
      pc->colors[0] = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t));
      for( int32_t i=0; i<n_pts; ++i ) {pc->colors[0][i] = msh_vec3( 0.5f, 0.5f, 0.5f ); }
    }
    else
    {
      pc->colors[0] = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t));
      for( int32_t i = 0; i < n_pts; ++i)
      {
        pc->colors[0][i] = msh_vec3( ((float)colors_tmp[3*i+0]) / 255.0f,
                                     ((float)colors_tmp[3*i+1]) / 255.0f,
                                     ((float)colors_tmp[3*i+2]) / 255.0f );
      }
      free( colors_tmp );
    }

    pc->qualities[0]    = (float*)malloc(n_pts*sizeof(float));
    for( int32_t i = 0; i < n_pts; ++i)
    {
      pc->qualities[0][i] = 1.0f;
    }
    msh_ply_close(pf);
  }

  return error;
}

int32_t
rs_pointcloud__save_ply( const char* filename, rs_pointcloud_t* pc, int32_t level )
{
  msh_ply_t *pf = msh_ply_open( filename, "wb" );
  int32_t error = MSH_PLY_FILE_OPEN_ERR;
  if( pf )
  {
    error = MSH_PLY_NO_ERR;
    const char* vertex_attributes[] = { "x", "y", "z",
                                        "nx", "ny", "nz",
                                        "red", "green", "blue",
                                        "radius", "class_idx", "instance_idx" };
    const char* face_attributes[] = { "vertex_indices" };

    uint8_t* colors_tmp = (uint8_t*)malloc( 3*pc->n_pts[level]*sizeof(uint8_t) );
    for( size_t i = 0; i < pc->n_pts[level]; ++i )
    {
      colors_tmp[3*i+0] = (uint8_t)(pc->colors[level][i].x * 255.0f );
      colors_tmp[3*i+1] = (uint8_t)(pc->colors[level][i].y * 255.0f );
      colors_tmp[3*i+2] = (uint8_t)(pc->colors[level][i].z * 255.0f );
    }

    msh_ply_desc_t positions_desc = { (char*)"vertex", &vertex_attributes[ 0], 3, MSH_PLY_FLOAT, MSH_PLY_INVALID, 
                                      &pc->positions[level], NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t normals_desc   = { (char*)"vertex", &vertex_attributes[ 3], 3, MSH_PLY_FLOAT, MSH_PLY_INVALID,
                                      &pc->normals[level], NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t colors_desc    = { (char*)"vertex", &vertex_attributes[ 6], 3, MSH_PLY_UINT8, MSH_PLY_INVALID, 
                                      &colors_tmp, NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t radii_desc     = { (char*)"vertex", &vertex_attributes[ 9], 1, MSH_PLY_FLOAT, MSH_PLY_INVALID, 
                                      &pc->radii[level], NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t class_desc     = { (char*)"vertex", &vertex_attributes[10], 1, MSH_PLY_INT32, MSH_PLY_INVALID, 
                                      &pc->class_ids[level], NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t instance_desc  = { (char*)"vertex", &vertex_attributes[11], 1, MSH_PLY_INT32, MSH_PLY_INVALID, 
                                      &pc->instance_ids[level], NULL, &pc->n_pts[level], 0 };
    msh_ply_desc_t face_desc      = { (char*)"face",   &face_attributes[0],    1, MSH_PLY_INT32, MSH_PLY_UINT8,   
                                      &pc->faces_ind, NULL, &pc->n_faces, 3 };

    if( pc->positions )    msh_ply_add_descriptor( pf, &positions_desc );
    if( pc->normals )      msh_ply_add_descriptor( pf, &normals_desc );
    if( pc->colors )       msh_ply_add_descriptor( pf, &colors_desc );
    if( pc->radii )        msh_ply_add_descriptor( pf, &radii_desc );
    if( pc->class_ids )    msh_ply_add_descriptor( pf, &class_desc );
    if( pc->instance_ids ) msh_ply_add_descriptor( pf, &instance_desc );
    if( pc->faces_ind )    msh_ply_add_descriptor( pf, &face_desc );

    error = msh_ply_write(pf);
    msh_cprintf( error, "%s\n", msh_ply_error_msg( error ) );

    free( colors_tmp );
  }

  msh_ply_close(pf);
  return error;
}


void
rs_pointcloud__compute_bbox(rs_pointcloud_t* pc)
{
  mshgeo_bbox_reset(&pc->bbox);
  for( size_t i = 0 ; i < pc->n_pts[0]; ++i)
  {
    mshgeo_bbox_union( &pc->bbox, pc->positions[0][i] );
  }
}

void
rs_pointcloud_compute_search_grid( rs_pointcloud_t* pc, int32_t level )
{
  assert( pc );
  assert( pc->voxel_size );

  int32_t n_pts = pc->n_pts[level];
  real32_t* pts = (real32_t*)&pc->positions[level][0].x;

  if( pc->search_grids[level] ) { msh_hash_grid_term( pc->search_grids[level] ); }
  free( pc->search_grids[level] );
  size_t hash_grid_size = sizeof(msh_hash_grid_t);
  pc->search_grids[level] = (msh_hash_grid_t*)calloc( 1, hash_grid_size );
  msh_hash_grid_init_3d( pc->search_grids[level], pts, n_pts, 0.05f );
}

void
rs_pointcloud__allocate_level( rs_pointcloud_t* pc, int32_t level, int32_t n_pts )
{
  pc->n_pts[level]        = n_pts;
  pc->positions[level]    = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t) );
  pc->normals[level]      = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t) );
  pc->colors[level]       = (msh_vec3_t*)malloc( n_pts * sizeof(msh_vec3_t) );
  pc->radii[level]        = (float*)malloc( n_pts * sizeof(float) );
  pc->qualities[level]    = (float*)malloc( n_pts * sizeof(float) );
  pc->class_ids[level]    = (int32_t*)malloc( n_pts * sizeof(int32_t) );
  pc->instance_ids[level] = (int32_t*)malloc( n_pts * sizeof(int32_t) );
}

void
rs_pointcloud__free_level( rs_pointcloud_t* pc, int32_t level )
{
  free( pc->positions[level] );    pc->positions[level] = NULL;
  free( pc->normals[level] );      pc->normals[level] = NULL;
  free( pc->colors[level] );       pc->colors[level] = NULL;
  free( pc->radii[level] );        pc->radii[level] = NULL;
  free( pc->qualities[level] );    pc->qualities[level] = NULL;
  free( pc->class_ids[level] );    pc->class_ids[level] = NULL;
  free( pc->instance_ids[level] ); pc->instance_ids[level] = NULL;

  if( pc->search_grids[level] )
  { 
    msh_hash_grid_term( pc->search_grids[level] );
  }
  free( pc->search_grids[level] ); pc->search_grids[level] = NULL;
  pc->n_pts[level] = 0;

  if( level == 0 )
  {
    free( pc->faces_ind );
    pc->n_faces = 0;
  }
}

void
rs_pointcloud__compute_level_grid(rs_pointcloud_t* pc, int32_t level )
{
  
  // Step 1. Get info on the top level
  msh_bbox_t bbox           = pc->bbox;
  int32_t n_pts             = pc->n_pts[0];
  msh_vec3_t* top_level_pos = pc->positions[0];
  msh_vec3_t* top_level_nor = pc->normals[0];
  msh_vec3_t* top_level_col = pc->colors[0];
  int32_t* top_level_cid    = pc->class_ids[0];
  int32_t* top_level_iid    = pc->instance_ids[0];
  float voxel_size          = pc->voxel_size[level];

  // Step 2. Bin the points using hashtable.
  int32_t x_res = (int32_t)ceil( mshgeo_bbox_width(&bbox) / voxel_size );
  int32_t y_res = (int32_t)ceil( mshgeo_bbox_height(&bbox) / voxel_size );

  hashtable_t table;
  hashtable_init(&table, sizeof(uintptr_t), 1024, NULL);
  for(int32_t i = 0; i < n_pts; ++i)
  {

    msh_vec3_t pos = top_level_pos[i];
    uint64_t idx_x = (uint64_t)floor( (pos.x - bbox.min_p.x) / voxel_size );
    uint64_t idx_y = (uint64_t)floor( (pos.y - bbox.min_p.y) / voxel_size );
    uint64_t idx_z = (uint64_t)floor( (pos.z - bbox.min_p.z) / voxel_size );
    uint64_t idx = idx_z * y_res * x_res + idx_y * x_res + idx_x;

    msh_array(int32_t)* val = (msh_array(int32_t)*)hashtable_find( &table,
                                                                   (HASHTABLE_U64)idx );
    if( val )
    {
      msh_array_push( *val, i );
    }
    else
    {
      msh_array(int32_t) indices = NULL;
      msh_array_push( indices, i );
      hashtable_insert( &table, idx, &indices );
    }
  }

  // Step 3. Copy the pointclouds by averaging bin contents
  int32_t count = hashtable_count( &table );
  rs_pointcloud__free_level( pc, level );
  rs_pointcloud__allocate_level( pc, level, count );
  msh_array(int32_t)* items = (msh_array(int32_t)*)hashtable_items( &table );
  int32_t pts_count_check = 0;
  for( int32_t i = 0; i < count; ++i )
  {
    msh_array(int32_t) indices = items[i];
    msh_vec3_t pos = msh_vec3_zeros();
    msh_vec3_t nor = msh_vec3_zeros();
    msh_vec3_t col = msh_vec3_zeros();
    float prp[1024] = {0};
    pts_count_check += msh_array_len( indices );
    for( size_t j = 0; j < msh_array_len( indices ); ++j )
    {
      pos = msh_vec3_add( pos, top_level_pos[indices[j]] );
      nor = msh_vec3_add( nor, top_level_nor[indices[j]] );
      col = msh_vec3_add( col, top_level_col[indices[j]] );
    }
    float s = 1.0f / (float)msh_array_len( indices );
    pc->positions[level][i]    = msh_vec3_scalar_mul( pos, s );
    pc->normals[level][i]      = msh_vec3_normalize( msh_vec3_scalar_mul( nor, s ) );
    pc->colors[level][i]       = msh_vec3_scalar_mul( col, s );
    pc->radii[level][i]        = 0.5f * voxel_size;
    pc->class_ids[level][i]    = top_level_cid[ indices[0] ];
    pc->instance_ids[level][i] = top_level_iid[ indices[0] ];
    pc->qualities[level][i]    = 0;
  }

  // cleanup
  for( int32_t i = 0; i < count; ++i )
  {
    msh_array_free(items[i]);
  }
  hashtable_term( &table );
}

void
rs_pointcloud__compute_level_poisson( rs_pointcloud_t* pc, int32_t level )
{
  // Create subsample grid from the top level
  msh_hash_grid_t subsample_grid = {0};
  msh_hash_grid_init_3d( &subsample_grid, (float*)pc->positions[0], 
                         pc->n_pts[0], 2.5f * pc->voxel_size[ level ] );

  size_t n_marked = 0;
  msh_array(int32_t) sample_ind = 0;
  int32_t last_sample_idx = 0;
  size_t max_n_neigh = 1024 * ((level) / (float)(RSPC_N_LEVELS-1));
  if( !max_n_neigh ) { max_n_neigh = 256; }
  size_t n_pts = pc->n_pts[0];

  int8_t* unmarked      = (int8_t*)malloc( n_pts * sizeof(int8_t));
  float* search_dist_sq = (float*)malloc( max_n_neigh * sizeof(float) );
  int32_t* search_ind       = (int32_t*)malloc( max_n_neigh * sizeof(int32_t) );
  for( size_t i = 0; i < n_pts; ++i ) 
  { 
    unmarked[i] = 1;
  }

  msh_hash_grid_search_desc_t grid_search_opts = {0};
  grid_search_opts.max_n_neigh  = max_n_neigh;
  grid_search_opts.distances_sq = search_dist_sq;
  grid_search_opts.indices      = search_ind;
  grid_search_opts.radius       = pc->voxel_size[ level ];
  grid_search_opts.sort         = 0;
  grid_search_opts.n_query_pts  = 1;
  
  while( n_marked < n_pts )
  {
    // select first unmarked point.
    int32_t sample_idx = last_sample_idx;
    while( unmarked[ sample_idx ] != 1 ) { sample_idx++; }
    last_sample_idx = sample_idx;

    // add it to set of samples
    msh_array_push( sample_ind, sample_idx );

    // mark all pts within a radius & update sampling distribution
    grid_search_opts.query_pts = (float*)(&pc->positions[0][ sample_idx ]);
    
    size_t n_neigh = msh_hash_grid_radius_search( &subsample_grid, &grid_search_opts );
    
    size_t n_valid_neigh = 0;
    for( size_t i = 0 ; i < n_neigh; ++i )
    {
      if( unmarked[ search_ind[i] ] > 0 ) { n_valid_neigh++; }
      unmarked[ search_ind[i] ] = 0;
    }
    n_marked += n_valid_neigh;
  }

  free( unmarked );
  free( search_dist_sq );
  free( search_ind );

  msh_vec3_t* positions = 0;
  msh_vec3_t* normals = 0;
  msh_vec3_t* colors = 0;
  float* radii = 0;
  float* qualities = 0;
  int32_t* classes = 0;
  int32_t* instances = 0;
  if( level > 0 )
  {
    positions = pc->positions[0];
    normals   = pc->normals[0];
    colors    = pc->colors[0];
    radii     = pc->radii[0];
    qualities = pc->qualities[0];
    classes   = pc->class_ids[0];
    instances = pc->instance_ids[0];
  }
  else
  {
    // If we are working with lvl 0, we need a copy
    positions = (msh_vec3_t*)malloc(pc->n_pts[0] * sizeof(msh_vec3_t));
    normals   = (msh_vec3_t*)malloc(pc->n_pts[0] * sizeof(msh_vec3_t));
    colors    = (msh_vec3_t*)malloc(pc->n_pts[0] * sizeof(msh_vec3_t));
    radii     = (float*)malloc(pc->n_pts[0] * sizeof(float));
    qualities = (float*)malloc(pc->n_pts[0] * sizeof(float));
    classes   = (int32_t*)malloc(pc->n_pts[0] * sizeof(int32_t));
    instances = (int32_t*)malloc(pc->n_pts[0] * sizeof(int32_t));

    memcpy( positions, pc->positions[0], pc->n_pts[0] * sizeof(msh_vec3_t));
    memcpy( normals, pc->normals[0], pc->n_pts[0] * sizeof(msh_vec3_t));
    memcpy( colors, pc->colors[0], pc->n_pts[0] * sizeof(msh_vec3_t));
    memcpy( radii, pc->radii[0], pc->n_pts[0] * sizeof(float));
    memcpy( qualities, pc->qualities[0], pc->n_pts[0] * sizeof(float));
    memcpy( classes, pc->class_ids[0], pc->n_pts[0] * sizeof(int32_t));
    memcpy( instances, pc->instance_ids[0], pc->n_pts[0] * sizeof(int32_t));
  }

  rs_pointcloud__free_level( pc, level );
  rs_pointcloud__allocate_level( pc, level, msh_array_len(sample_ind) );

  for( size_t i = 0; i < msh_array_len(sample_ind); ++i )
  {
    if( positions) pc->positions[level][i]     = positions[sample_ind[i]];
    if( normals ) pc->normals[level][i]        = normals[sample_ind[i]];
    if( colors ) pc->colors[level][i]          = colors[sample_ind[i]];
    if( radii ) pc->radii[level][i]            = radii[sample_ind[i]];
    if( qualities ) pc->qualities[level][i]    = qualities[sample_ind[i]];
    if( classes ) pc->class_ids[level][i]      = classes[sample_ind[i]];
    if( instances ) pc->instance_ids[level][i] = instances[sample_ind[i]];
  }

  if( level == 0 )
  {
    free(positions);
    free(normals);
    free(colors);
    free(radii);
    free(qualities);
    free(classes);
    free(instances);
  }

  msh_array_free(sample_ind);
}

void
rs_pointcloud__compute_level(rs_pointcloud_t* pc, int32_t level )
{
  rs_pointcloud__compute_level_poisson( pc, level );
}

inline msh_vec3_t
rs_pointcloud__random_barycentric_coords( msh_rand_ctx_t* rand_gen )
{

  double  s = msh_rand_nextf( rand_gen );
  double  t = msh_rand_nextf( rand_gen );

  if( s + t > 1.0 )
  {
    s = 1.0 - s;
    t = 1.0 - t;
  }

  double q = 1.0 - s - t;
  msh_vec3_t v = msh_vec3( (float)q, (float)s, (float)t );
  return v;
}

void
rs_pointcloud_uniform_resample( const rs_pointcloud_t* in_mesh, rs_pointcloud_t* out_pc )
{
  uint64_t seeds[2] = { 12346ULL, 64321ULL };
  msh_rand_ctx_t rand_gen = {0};
  msh_rand_init( &rand_gen, seeds[0] );

  double* areas = (double*)malloc( in_mesh->n_faces * sizeof(double) );
  double total_area = 0;

  // Compute triangle areas
  for( size_t i = 0; i < in_mesh->n_faces; i++ )
  {
    msh_vec3_t a = in_mesh->positions[0][in_mesh->faces_ind[3 * i + 0]];
    msh_vec3_t b = in_mesh->positions[0][in_mesh->faces_ind[3 * i + 1]];
    msh_vec3_t c = in_mesh->positions[0][in_mesh->faces_ind[3 * i + 2]];

    msh_vec3_t v1 = msh_vec3_sub( b, a );
    msh_vec3_t v2 = msh_vec3_sub( c, a );
    double cur_area = msh_vec3_norm( msh_vec3_cross( v1, v2 ) );
    total_area += cur_area;
    areas[ i ] = cur_area;
  }

  // Prepare data needed for sampling
  const double n_samples_per_square_meter = 12800;
  size_t n_samples = 0.5 * total_area * n_samples_per_square_meter;
  msh_discrete_distrib_t sampling_ctx = {0};
  msh_discrete_distribution_init( &sampling_ctx, areas, in_mesh->n_faces, seeds[1] );

  // Prepare storage
  rs_pointcloud__free_level( out_pc, 0 );
  rs_pointcloud__allocate_level( out_pc, 0, n_samples );

  // Start sampling
  for( size_t i = 0 ; i < n_samples; ++i )
  {
    // Select random face
    size_t fi = (size_t)msh_discrete_distribution_sample( &sampling_ctx );

    // Select barycentric coords
    msh_vec3_t coords = rs_pointcloud__random_barycentric_coords( &rand_gen );

    int32_t vi[3] = { in_mesh->faces_ind[3 * fi + 0],
                  in_mesh->faces_ind[3 * fi + 1],
                  in_mesh->faces_ind[3 * fi + 2] };

    // Compute attributes of sampled point from barycentric coords
    msh_vec3_t pa = msh_vec3_scalar_mul( in_mesh->positions[0][vi[0]], coords.data[0] );
    msh_vec3_t pb = msh_vec3_scalar_mul( in_mesh->positions[0][vi[1]], coords.data[1] );
    msh_vec3_t pc = msh_vec3_scalar_mul( in_mesh->positions[0][vi[2]], coords.data[2] );
    out_pc->positions[0][i] = msh_vec3_add( msh_vec3_add(pa, pb), pc);

    msh_vec3_t na = msh_vec3_scalar_mul( in_mesh->normals[0][vi[0]], coords.data[0] );
    msh_vec3_t nb = msh_vec3_scalar_mul( in_mesh->normals[0][vi[1]], coords.data[1] );
    msh_vec3_t nc = msh_vec3_scalar_mul( in_mesh->normals[0][vi[2]], coords.data[2] );
    out_pc->normals[0][i] = msh_vec3_normalize( msh_vec3_add( msh_vec3_add(na, nb), nc) );

    msh_vec3_t ca = msh_vec3_scalar_mul( in_mesh->colors[0][vi[0]], coords.data[0] );
    msh_vec3_t cb = msh_vec3_scalar_mul( in_mesh->colors[0][vi[1]], coords.data[1] );
    msh_vec3_t cc = msh_vec3_scalar_mul( in_mesh->colors[0][vi[2]], coords.data[2] );
    out_pc->colors[0][i] = msh_vec3_add( msh_vec3_add(ca, cb), cc);

    double ra = in_mesh->radii[0][vi[0]] * coords.data[0];
    double rb = in_mesh->radii[0][vi[1]] * coords.data[1];
    double rc = in_mesh->radii[0][vi[2]] * coords.data[2];
    out_pc->radii[0][i] = ra + rb + rc;

    double min_dist = msh_min3( coords.x, coords.y, coords.z );

    int32_t cia = in_mesh->class_ids[0][vi[0]];
    int32_t cib = in_mesh->class_ids[0][vi[1]];
    int32_t cic = in_mesh->class_ids[0][vi[2]];
    int32_t iia = in_mesh->instance_ids[0][vi[0]];
    int32_t iib = in_mesh->instance_ids[0][vi[1]];
    int32_t iic = in_mesh->instance_ids[0][vi[2]];
    if( coords.x == min_dist )
    {
      out_pc->class_ids[0][i]    = cia;
      out_pc->instance_ids[0][i] = iia;
    }
    else if( coords.y == min_dist )
    {
      out_pc->class_ids[0][i]    = cib;
      out_pc->instance_ids[0][i] = iib;
    }
    else
    {
      out_pc->class_ids[0][i] = cic;
      out_pc->instance_ids[0][i] = iic;
    }
  }

  free( areas );
  msh_discrete_distribution_free( &sampling_ctx );
}

char* rs_pointcloud__get_json_string( const char* filename )
{
  FILE* file = fopen( filename, "r" );
  if( !file ) { return NULL; }

  fseek( file, 0, SEEK_END );
  uint64_t fsize = ftell( file );
  fseek( file, 0, SEEK_SET );

  char *string = (char*) malloc(fsize + 1);
  fread( string, fsize, 1, file );
  fclose( file );

  string[fsize] = 0;

  return string;
}

int32_t
rs_pointcloud_from_files( rs_pointcloud_t* pcs, const char** filenames, int32_t n, int32_t verbose_lvl )
{
  msh_cprintf( verbose_lvl, "IO: Loading and processing %d pointclouds...\n", n);
  uint64_t gt1 = msh_time_now();

  for( int32_t i = 0 ; i < n; ++i )
  {
    uint64_t t1, t2;
    const char* filename = filenames[i];
    rs_pointcloud_t* load_file = rs_pointcloud_init(1);
    msh_cprintf(verbose_lvl, "IO: Loading file %d - %s\n", i, filename );

    t1 = msh_time_now();
    int32_t error = rs_pointcloud__load_ply( filename, load_file, 0 );
    if( error ) { printf("Error loading file %s\n", filename); return error; }
    t2 = msh_time_now();

    msh_cprintf( verbose_lvl > 0, "   |  Loading .ply took %fms\n",
                 msh_time_diff_ms(t2, t1));

    if( load_file->n_faces > 0 )
    {
      t1 = msh_time_now();
      rs_pointcloud_uniform_resample( load_file, &pcs[i] );
      t2 = msh_time_now();
      msh_cprintf( verbose_lvl > 0, "   |  Resampling (%d->%d) took %fms\n", 
                   (int32_t)load_file->n_pts[0], (int32_t)pcs[i].n_pts[0], msh_time_diff_ms(t2, t1) );
      rs_pointcloud_free( load_file, 1 );
    }
    else
    {
      memcpy( &pcs[i], load_file, sizeof(rs_pointcloud_t) );
      free( load_file );
    }

    t1 = msh_time_now();
    rs_pointcloud_compute_levels( &pcs[i] );
    t2 = msh_time_now();
    msh_cprintf(verbose_lvl > 0, "   |  Computing levels took %fs.\n", msh_time_diff_sec(t2, t1));
  }
  uint64_t gt2 = msh_time_now();
  msh_cprintf( verbose_lvl, "IO: Done in %fs.\n", msh_time_diff_sec(gt2, gt1));
  return 0;
}

int32_t
rs_pointcloud_from_file(rs_pointcloud_t* pc, const char* filename)
{
  return rs_pointcloud_from_files( pc, &filename, 1, 0 );
}

void
rs_pointcloud_to_file( rs_pointcloud_t* pc, const char* filename, int32_t lvl )
{
  rs_pointcloud__save_ply( filename, pc, lvl );
}

void
rs_pointcloud_compute_levels( rs_pointcloud_t* pc )
{
  rs_pointcloud__compute_bbox( pc );
  pc->centroid = msh_vec3( -MSH_F32_MAX, -MSH_F32_MAX, -MSH_F32_MAX);
  pc->covariance.data[0] = -MSH_F32_MAX;
  for( int32_t j = 0; j < RSPC_N_LEVELS; ++j )
  {
    if( j > 0 ) { rs_pointcloud__compute_level( pc, j ); }
    rs_pointcloud_compute_search_grid( pc, j );
  }
}

msh_vec3_t
rs_pointcloud_centroid( rs_pointcloud_t* pc, int32_t lvl )
{
  if( pc->centroid.x > -MSH_F32_MAX )
  {
    return pc->centroid;
  }
  
  double c[3] = {0, 0, 0};
  for( size_t i = 0; i < pc->n_pts[lvl]; ++i )
  {
    c[0] += pc->positions[lvl][i].x;
    c[1] += pc->positions[lvl][i].y;
    c[2] += pc->positions[lvl][i].z;
  }
  c[0] /= (double)pc->n_pts[lvl];
  c[1] /= (double)pc->n_pts[lvl];
  c[2] /= (double)pc->n_pts[lvl];
  pc->centroid = msh_vec3( (float)c[0], (float)c[1], (float)c[2] );

  return pc->centroid;
}

msh_mat3_t
rs_pointcloud_covariance( rs_pointcloud_t* pc, int32_t lvl )
{
  if( pc->covariance.data[0] > -MSH_F32_MAX )
  {
    return pc->covariance;
  }

  pc->covariance = mshgeo_pts3d_covariance( rs_pointcloud_centroid(pc, lvl), pc->positions[lvl], pc->n_pts[lvl] );
  
  return pc->covariance;
}

void
rs_pointcloud_translate( rs_pointcloud_t* pc, msh_vec3_t t, int32_t compute_levels )
{
  pc->centroid = msh_vec3( -MSH_F32_MAX, -MSH_F32_MAX, -MSH_F32_MAX);

  for( size_t i = 0; i < pc->n_pts[0]; ++i )
  {
    pc->positions[0][i] = msh_vec3_add(pc->positions[0][i], t );
  }
  if(compute_levels) rs_pointcloud_compute_levels(pc);
}


void
rs_pointcloud_transform( rs_pointcloud_t* pc, msh_mat4_t xform, int32_t compute_levels )
{
  pc->centroid = msh_vec3( -MSH_F32_MAX, -MSH_F32_MAX, -MSH_F32_MAX);
  pc->covariance.data[0] = -MSH_F32_MAX;
  for( size_t i = 0; i < pc->n_pts[0]; ++i )
  {
    pc->positions[0][i] = msh_mat4_vec3_mul(xform, pc->positions[0][i], 1 );
    pc->normals[0][i]   = msh_mat4_vec3_mul(xform, pc->normals[0][i], 0 );
  }
  if(compute_levels) rs_pointcloud_compute_levels(pc);
}

#endif /*RS_POINTCLOUD_IMPLEMENTATION*/