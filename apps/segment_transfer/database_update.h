#ifndef RS_DATABASE_UPDATE_H
#define RS_DATABASE_UPDATE_H


void
rsdu_augment_database( rsdb_t* rsdb, rs_pointcloud_t* input_scan,
                       msh_array(rs_obj_plcmnt_t) arrangement );

#endif /*RS_DATABASE_UPDATE_H*/
