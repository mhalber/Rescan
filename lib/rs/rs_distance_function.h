
/*
  ==============================================================================
  
  RS_DISTANCE_FIELD.H - WIP! 
  
  This file contains implementation for distance functions

  #define RS_DISTANCE_FIELD_IMPLEMENTATION
  #include "rs_distance_field.h"

  ==============================================================================
  DEPENDENCIES
  msh.h, msh_vec3.h

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
  [ ] Error codes and strings
  [ ] Truncated distance field
  [ ] Add more elegant index to position / position to index conversion functions.
  ==============================================================================
  REFERENCES:
*/

#ifndef RS_DISTANCE_FIELD_H
#define RS_DISTANCE_FIELD_H

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct rs_tdf
// {
//  // TODO(maciej): A lot of space will be zero. TSDF should be sparse
// } rs_tdf_t;

typedef struct rs_df
{
  float* data;
  int32_t xres;
  int32_t yres;
  int32_t zres;
  int32_t ncells;
  float voxel_size;
  msh_bbox_t bbox;
} rs_df_t;

// NOTE(maciej): I think I want the data to be initialized this way. Can initialize multiple
// objects as well as return error codes.
int rs_df_init(rs_df_t* df, float* pos, int32_t npos, float voxel_size);
int rs_df_free(rs_df_t* df);

float rs_df_closest_surface(rs_df_t* df, float* pos);


#ifdef __cplusplus
}
#endif

#endif /*RS_DISTANCE_FUNCTION_H*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef RS_DISTANCE_FUNCTION_IMPLEMENTATION


int
rs_df_init( rs_df_t* df, float* posns, int32_t npos, float voxel_size )
{
  if( voxel_size <= 0.0000001 ) return 1;
  if( npos <= 0 ) return 1;
  if( !posns ) return 1;

  // construct bbox
  mshgeo_bbox_reset( &df->bbox );
  float* posns_ptr = posns;
  for(int i = 0 ; i < npos; ++i)
  {
    msh_vec3_t* pos = (msh_vec3_t*)posns_ptr;
    mshgeo_bbox_union(&df->bbox, *pos);
    posns_ptr+=3;
  }

  df->voxel_size = voxel_size;
  df->xres = (int32_t)ceil(mshgeo_bbox_width(&df->bbox) / voxel_size);
  df->yres = (int32_t)ceil(mshgeo_bbox_height(&df->bbox) / voxel_size);
  df->zres = (int32_t)ceil(mshgeo_bbox_depth(&df->bbox) / voxel_size);
  df->ncells = df->xres*df->yres*df->zres;
  df->data = (float*)malloc(df->ncells*sizeof(float));
  if(!df->data) return 1;

  // build search index
  // flann::Matrix<float> pts_mat((float*)&posns[0], npos, 3);
  // flann::Index<flann::L2_Simple<float>>* search_index;
  // search_index = new flann::Index<flann::L2_Simple<float>>(pts_mat, 
  //                                                          flann::KDTreeSingleIndexParams(16));
  // search_index->buildIndex();

  // compute signed distance field
  // TODO(maciej): Needs to be multithreaded
  // float* sdf_data_ptr = df->data;
  // for(int z = 0; z < df->zres; z++)
  // {
  //   for(int y = 0; y < df->yres; y++)
  //   {
  //     for(int x = 0; x < df->xres; x++)
  //     {
  //       msh_vec3_t voxel_center = df->bbox.min_p + msh_vec3( (x + 0.5f) * voxel_size, 
  //                                                            (y + 0.5f) * voxel_size,
  //                                                            (z + 0.5f) * voxel_size );
  //       float *query = &voxel_center.data[0];
  //       int idx = -1; float dist = 1e9;
  //       flann::Matrix<float> query_mat(&query[0], 1, 3);
  //       flann::Matrix<int> indices_mat(&idx, 1, 1);
  //       flann::Matrix<float> dists_mat(&dist, 1, 1);
  //       search_index->knnSearch(query_mat, indices_mat, dists_mat, 
  //                               1, flann::SearchParams(128) );
  //       *sdf_data_ptr = dist;
  //       sdf_data_ptr++;
  //     }
  //   }
  // }
  // delete search_index;

  return 0;
}

int rs_df_free(rs_df_t* df)
{
  df->xres = -1;
  df->yres = -1;
  df->zres = -1;
  mshgeo_bbox_reset(&df->bbox);
  free(df->data);
  return 0;
}

float rs_df_closest_surface(rs_df_t* df, float* pos)
{
  if( !df ) return -1.0f;
  if( !pos ) return -1.0f;
  int32_t x = (int32_t)floor((pos[0]-df->bbox.min_p.x) / df->voxel_size);
  int32_t y = (int32_t)floor((pos[1]-df->bbox.min_p.y) / df->voxel_size);
  int32_t z = (int32_t)floor((pos[2]-df->bbox.min_p.z) / df->voxel_size);
  int32_t idx = msh_max(0, z * df->yres*df->xres + y * df->xres + x);
  return df->data[idx];
}

#endif /*RS_DISTANCE_FUNCTION_IMPLEMENTATION*/