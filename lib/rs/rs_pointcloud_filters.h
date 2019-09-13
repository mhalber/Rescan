#ifndef RS_POINTCLOUD_FILTERS_H
#define RS_POINTCLOUD_FILTERS_H


/* PLANES */
typedef struct rspf_edge
{
  int idx1;
  int idx2;
  float weight;
} rspf_edge_t;


typedef struct rspf_plane_feature_set
{
  size_t count;
  float max_y;
  float normal_up_dot;
  float saliency;
} rspf_plane_feature_set_t;

typedef struct rspf_plane_model
{
  msh_plane_t plane;
  size_t n_inliers;
  msh_array( int32_t ) inlier_ind;
  msh_mat3_t axes;
  msh_vec4_t extends;
  int8_t valid;
  rspf_plane_feature_set_t features;
} rspf_plane_model_t;

void
rspf_detect_planes( const rs_pointcloud_t* pc, 
                    msh_array( rspf_plane_model_t) *planes );

void
rspf_compute_plane_features( const rs_pointcloud_t* pc, 
                             msh_array( rspf_plane_model_t) *planes );

void
rspf_classify_planes( const rs_pointcloud_t* pc, 
                      msh_array( rspf_plane_model_t) *planes );

void
rspf_relabel_walls_and_floors( const rsdb_t* rsdb,
                               const rs_pointcloud_t *pc,
                               msh_array( rspf_plane_model_t) *planes );

/* LABELS */
typedef struct rspf_smooth_options
{
  uint32_t dynamic_object_cost;
  uint32_t static_object_cost;
  uint32_t edge_cost;
} rspf_smooth_options_t;

void 
rspf_smooth_labels( rsdb_t* rsdb, 
                    rs_pointcloud_t* in_pc );

msh_array(rspf_edge_t)
rspf_compute_neighborhood( const rs_pointcloud_t* pc, 
                           int lvl, 
                           int max_nn, 
                           float radius_sq,
                           float dist_exp,
                           float angle_exp );
void
rspf_arrangement_to_labels( rsdb_t* rsdb, 
                            rs_pointcloud_t* in_pc, 
                            msh_array(rs_obj_plcmnt_t) arrangement,
                            float radius,
                            bool prioritize_static );

#endif /*RS_POINTCLOUD_FILTERS_H*/
