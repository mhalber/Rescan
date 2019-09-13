#define ICP_IMPLEMENTATION

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "msh/msh_std.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_hash_grid.h"
#include "mg/hashtable.h"
#include "rs_pointcloud.h"
#include "rs_distance_function.h"
#include "rs_database.h"
#include "icp.h"

#include "GCoptimization.h" 

#include "database_update.h"

void 
rsdu_augment_database( rsdb_t* rsdb, rs_pointcloud_t* input_scan, 
                       msh_array(rs_obj_plcmnt_t) arrangement )
{
  assert( msh_array_len(rsdb->arrangements) >= 2 );

  for( size_t cur_idx = 0; cur_idx < msh_array_len( arrangement ); ++cur_idx )
  {
    int novel_object = 0;
    rs_obj_plcmnt_t* cur_placement = &arrangement[cur_idx];
    rs_object_t* cur_object = &rsdb->objects[ cur_placement->object_idx ];
    
    // Extract shape with given uidx.
    rs_pointcloud_t* extracted_shape = rs_pointcloud_copy_by_ids( input_scan, 1, RS_PT_INSTANCE_ID, &cur_placement->uidx, 1, 0 );
  
    printf("DATABASE_AUGMENT: Working on placement %3d - %s | (%3d/%3d)\n",
            (int32_t)cur_placement->uidx, cur_object->filename,
            (int32_t)cur_idx, (int32_t)msh_array_len( arrangement ) );
    if( cur_placement->uidx != cur_object->uidx )
    {
      novel_object = 1;
    }
    if( novel_object )
    {
      rs_object_t new_object = *cur_object;
      new_object.uidx = cur_placement->uidx;
      char buf[1024];
      snprintf(buf, 1024, "%s.%03d.ply", rsdb_get_class_name(rsdb, new_object.class_idx), new_object.uidx);
      new_object.filename = msh_strdup(buf);
      new_object.shape = rs_pointcloud_copy( cur_object->shape );
      
      cur_placement->object_idx = rsdb_add_object( rsdb, &new_object );
      cur_object = &rsdb->objects[ cur_placement->object_idx ];
      printf("DATABASE_AUGMENT:  --- Novel object %s!\n",  new_object.filename );
    }

    if( extracted_shape )
    {
      rs_pointcloud_t* cur_shape = cur_object->shape;
      
      msh_mat4_t xform = msh_mat4_inverse( cur_placement->pose );
      if( !rsdb_is_object_static( rsdb, cur_placement->object_idx ) )
      {
        icp_align( extracted_shape->positions[0], extracted_shape->normals[0], extracted_shape->n_pts[0],
                  cur_shape->positions[0], cur_shape->normals[0], cur_shape->n_pts[0],
                  &xform, msh_mat4_identity(), 0.05, msh_deg2rad(10.0f), 0);
      }
      rs_pointcloud_transform( extracted_shape, xform, 0 );
      for( size_t i = 0; i < extracted_shape->n_pts[0]; ++i )
      {
        extracted_shape->instance_ids[0][i] = 0;
      }
      for( size_t i = 0; i < cur_shape->n_pts[0]; ++i )
      {
        cur_shape->instance_ids[0][i] = 1;
      }
      rs_pointcloud_t* merged_shape = rs_pointcloud_merge( extracted_shape, cur_shape, 0 );
      for( size_t lvl = 0; lvl < RSPC_N_LEVELS; ++lvl )
      {
        for( size_t i = 0; i < merged_shape->n_pts[lvl]; ++i )
        {
          merged_shape->instance_ids[lvl][i] = cur_placement->uidx ;
        }
      }
      rs_pointcloud_free( cur_shape, 1 );
      rs_pointcloud_free( extracted_shape, 1 );

      cur_object->shape = merged_shape;
    }
  }
}