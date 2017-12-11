/**

* @file       had_interface.h

* @brief      Provide functions to get the HAD Data

* @author

* @date       2017-04-05

*/

#ifndef LOCALIZATION_HAD_INTERFACE_H
#define LOCALIZATION_HAD_INTERFACE_H

#include <vector>
#include <set>
#include "had_feature_define.h"

namespace had {
class IHadDB {
public:
    /**

     *

     *     initialize, set database path

     *     @param dbpath: the database full path

     *     @return   if true, open the database success

    */
    virtual bool initialize(const std::string dbpath) = 0;


    /**
    *
    *       initialize the start position: after initialize the database, start position must be set

    *       @param gps: the start position

    *       @return    NULL

    */
    virtual void init_start_position(const had::GPS_Coord gps) = 0;

    /**

     *

     *     get all the link's info from database

     *     @param p_link_vec: output the link information

     *     @return    if true, find at least one record

    */
    virtual bool get_link_info(
        std::vector<had::Link*>* p_link_vec) = 0;

    /**

     *

     *     get link's info based on linkID

     *     @param id_set: input the road ID set

     *     @param p_link_vec: output the link information

     *     @return    if true, find at least one record

    */
    virtual bool get_link_info(
        const std::set<int> id_set,
        std::vector<had::Link*>* p_link_vec) = 0;

    /**

     *

     *     get all the lane's info from database

     *     @param id_set: p_lane_vec: output the lane information

     *     @return    if true, find at least one record

    */
    virtual bool get_lane_info(
        std::vector<had::Lane*>* p_lane_vec) = 0;

    /**

     *

     *     get the lane's info based on LaneId or RoadID

     *     @param bflag_road: true, input link's ID; false, input lane's ID

     *     @param id_set:     input the linkID set or laneID set

     *     @param p_lane_vec: output the lane information

     *     @return    if true, find at least one record

    */
    virtual bool get_lane_info(
        const bool bflag_road,
        const std::set<int> id_set,
        std::vector<had::Lane*>* p_lane_vec) = 0;

    /**

     *

     *     get all the link's topology from database

     *     @param p_link_topology_vec: output the link topology information

     *     @return    if true, find at least one record

    */
    virtual bool get_link_topology(
        std::vector<had::LinkTopology*>* p_link_topology_vec) = 0;

    /**

     *

     *     get the link's topology base on RoadID

     *     @param id_set:                 input the road ID set

     *     @param p_link_topology_vec:    output the link topology information

     *     @return    if true, find at least one record

    */
    virtual bool get_link_topology(
        const std::set<int> id_set,
        std::vector<had::LinkTopology*>* p_link_topology_vec) = 0;

    /**

     *

     *     get all the lane's topology info from database

     *     @param p_lane_topology_vec: output the lane topology information

     *     @return    if true, find at least one record

    */
    virtual bool get_lane_topology(
        std::vector<had::LaneTopology*>* p_lane_topology_vec) = 0;

    /**

     *

     *     get the lane's topology info base on LaneID

     *     @param id_set:              input the lane ID set

     *     @param p_lane_topology_vec: output the lane topology information

     *     @return    if true, find at least one record

    */
    virtual bool get_lane_topology(
        const std::set<int> id_set,
        std::vector<had::LaneTopology*>* p_lane_topology_vec) = 0;

    /**

     *

     *     get all the objects info from database

     *     @param p_object_vec: output the objects information

     *     @return    if true, find at least one record

    */
    virtual bool get_objects_info(
        std::vector<had::Object*>* p_object_vec) = 0;

    /**

     *

     *     get the objects info base on objectID

     *     @param id_set:        input the objectID set

     *     @param p_object_vec:  output the objects information

     *     @return    if true, find at least one record

    */
    virtual bool get_objects_info(
        const std::set<int> id_set,
        std::vector<had::Object*>* p_object_vec) = 0;

    /**

     *

     *     according to position information, access to the roadID and LaneID

     *     @param gps:    input the position information

     *     @param laneid: output the laneid which the position located on

     *     @param linkid: output the linkid which the position located on

     *     @return    if true find at least one record

    */
    virtual bool matching(
        const had::GPS_Coord gps,
        int& laneid,
        int& linkid) = 0;

    /**

     *

     *     according to position information and radius, get neighbor link

     *     @param gps:        input the position information

     *     @param radius:     input the buffer radius(Units: meters)

     *     @param roadid_set: output the link ID which intersection with the buffer

     *     @return    if true find at least one record

    */
    virtual bool neighbor_search(
        const had::GPS_Coord gps,
        const double radius,
        std::set<int>& linkid_set) = 0;

    /**

    *      according to position information and radius, get the lane and object

    *      @param gps             :     input the position information

    *      @param radius          :     input the buffer radius(Units: meters)

    *      @param p_object_vec    :     The object info in the radius

    *      @param p_lane_info_vec :     The lane info in the radius

    *      @param p_lane_topo_vec :     The topology relation in the radius

    *      @return    None

    */
    virtual void get_around_map(
        const had::GPS_Coord gps,
        const double radius,
        std::vector<had::Object*>* p_object_vec,
        std::vector<had::Lane*>* p_lane_info_vec,
        std::vector<had::LaneTopology*>* p_lane_topo_vec,
        std::vector<had::Link*>* p_link_vec,
        std::vector<had::LinkTopology*>* p_link_topology_vec) = 0;

    /**

    *      get the guidance point traveled along the road

    *      @param gps             :     input the position information

    *      @param distance        :     input the distance traveled along the road  (Units: meters)

    *      @param p_guidance_point_vec :   return the guidance point in the path

    *      @return    if true find at least one record

    */
    virtual bool get_ahead_guidance_point(
        const had::GPS_Coord gps,
        const double distance,
        std::vector<had::Guidance_point*>* p_guidance_point_vec) = 0;

    /**

     *

     *     free road info

     *     @param p_Link_vec: the pointer need free

     *     @return    None

    */
    virtual void free_link_info(
        std::vector<had::Link*>* p_link_vec) = 0;

    /**

     *

     *     free road topology info

     *     @param p_link_topology_vec: the pointer need free

     *     @return    None

    */
    virtual void free_link_topo_info(
        std::vector<had::LinkTopology*>* p_link_topology_vec) = 0;

    /**

     *

     *     free lane info

     *     @param p_lane_vec: the pointer need free

     *     @return    None

    */
    virtual void free_lane_info(
        std::vector<had::Lane*>* p_lane_vec) = 0;

    /**

     *

     *     free lane topology info

     *     @param p_lane_topology_vec: the pointer need free

     *     @return    None

    */
    virtual void free_lane_topo_info(
        std::vector<had::LaneTopology*>* p_lane_topology_vec) = 0;

    /**

     *

     *     free object info

     *     @param p_object_vec: the pointer need free

     *     @return    None

    */
    virtual void free_obj_info(
        std::vector<had::Object*>* p_object_vec) = 0;


    /**

     *

     *     free object info

     *     @param p_object_vec: the pointer need free

     *     @return    None

    */
    virtual void free_ahead_guidance_point(
        std::vector<had::Guidance_point*>* p_guidance_point_vec) = 0;
};
}

/**

 *

 *     get the had interface pointer

 *     @return    had::IHadDB the had interface pointer

*/
had::IHadDB* get_haddb();

#endif //HAD_INTERFACE_H
