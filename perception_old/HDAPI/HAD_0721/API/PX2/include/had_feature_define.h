/**

* @file       had_feature_define.h

* @brief      Define the structure used by the interface

* @author     

* @date       2017-04-05
      
*/

#ifndef HAD_FEATURE_DEFINE_H
#define HAD_FEATURE_DEFINE_H

#include <string>
#include <vector>
#include <set>

namespace had
{    
    /**
    
     * Link's type
     
    */
    enum LinkType
    {
        LT_NO_SPECIAL             = 0,    //normal link
        LT_RAMP                   = 1    //ramp connect highway
    };
    
    /**
    
     * Link's Class
     
    */
    enum LinkClass
    {
        LC_EXPRESSWAY       = 1,  //highway
        LC_URBAN_EXPRESSWAY = 2,  //
        LC_NORMAL_LINK      = 3
    };
    
    /**
    
     * Link、Lane Direction
     
    */
    enum Direction
    {
        IN_POSITIVE_DIRECTION = 1,
        IN_NEGATIVE_DIRECTION = 2,
        IN_BOTH_DIRECTIONS = 3
    };
    
    /**
    
     * Attribute Code
     
    */    
    enum AttributeCode
    {
        MIN_SPEED_LIMIT = 0,
        MAX_SPEED_LIMIT = 1
    };
    
    /**
    
     * Attribute type
     
    */ 
    enum AttributeType
    {
        AT_NONE   = 0, ///< No Value
        AT_INT    = 1, ///< int
        AT_STRING = 2, ///< string
        AT_DOUBLE = 3  ///< double
    };
    
    /**
    
     * Attribute Value
     
    */ 
    union AttributeValue
    {
        int    vl_int;
        char   vl_char[255];
        double vl_double;
    };

    /**
    
     * Attribute information
     
    */ 
    struct AttributeInfo
    {
        AttributeCode  attr_code;
        AttributeType  attr_type;
        AttributeValue attr_value;
    };
    
    /**
    *
    *   Guidance Point Type
    *
    */
    enum GuidancePointType
    {
        GPT_ENTRY_CURVE     = 0,
        GPT_OUT_CURVE       = 1,
        GPT_MERGE           = 2, ///< one lane merge into other lane
        GPT_SPLIT           = 3, ///< new lane forming
        GPT_CONSTRUCT_START = 4,
        GPT_CONSTRUCT_END   = 5,
        GPT_ENTRY           = 6, ///< other road entry into the main road
        GPT_EXIT            = 7  ///< exit the main road to other road
    };
    
    /**
    *   Guidance_point
    */
    struct Guidance_point
    {
        GuidancePointType  pointtype;
        double distance;
        int linkid;
        int laneid; ///<if -1, all lanes of the linkid
    };
   
    /**
    * struct point3D
    *
    */
    struct Point3D
    {
        double x;
        double y;
        double z;
    };
    /**
    
     * Link information
     
    */ 
    struct Link
    {
        int            id;
        LinkClass      link_class;
        Direction      travel_direction;
        LinkType       link_type;
        int            lane_num;
        bool           b_tunnel;
        bool           b_bridge;
        bool           b_toll;
        //Units: meters
        double         length;
        int            speed_limit;
        int            vehiche_type;
        std::vector<Point3D> geometry;
    };
    
    /**
    
     * Connection between Links
     
    */ 
    struct LinkTopology
    {
        int linkid_from;
        int linkid_to;
        //intersection point
        Point3D junction_point;
    };

    /**
    
     * Lane type
     
    */ 
    enum LaneType
    {
        LAT_NORMAL      = 0,
        LAT_ENTRY       = 1,
        LAT_EXIT        = 2,
        LAT_EMERGENCY   = 3,
        LAT_ON_RAMP     = 4,
        LAT_OFF_RAMP    = 5,
    };

    /**
    
     * Lane boundary type
     
    */ 
    enum LaneMarkingType
    {
        LBR_MARKING_NONE                              = 0,
        LBR_MARKING_LONG_DASHED_LINE                  = 1,
        LBR_MARKING_DOUBLE_SOLID_LINE                 = 2,
        LBR_MARKING_SINGLE_SOLID_LINE                 = 3,
        LBR_MARKING_RIGHT_SOLID_LINE_LEFT_DASHED_LINE = 4,
        LBR_MARKING_LEFT_SOLID_LINE_RIGHT_DASHED_LINE = 5,
        LBR_MARKING_DOUBLE_DASHED_LINE                = 9,
        LBR_MARKING_CURB                              = 11,
        LBR_MARKING_GUARDRAIL                         = 17,
        LBR_MARKING_UNKNOWN                           = (-9999)
    };
    
    /**
    
     * Attribute Value
     
    */ 
    enum LaneMarkingMaterial
    {
        UN_KNOWN    = 0,
        METAL       = 1,
        CONCRETE    = 2,
        STONE       = 3,
        WOOD        = 4,
        PLASTIC     = 5,
        TRANSPARENT1 = 6,
        VIBRATION_MARKINGS        = 7,
        PAINTED_VIBRATION_DIVIDER = 8
    };
    
    /**
    
     * Lane boundary marking color
     
    */ 
    enum LaneMarkingColor
    {
        COLOR_NONE       = 0,
        COLOR_YELLOW     = 1,
        COLOR_WHITE      = 2,
        COLOR_BLUE       = 10,
        COLOR_ORANGE     = 11
    };
   
    /**
    
     * lane boundary information
     
    */ 
    struct LaneMarking
    {
        int lane_marking_id;
        int side;        ///< 1:on the lane's right, 0:left
        LaneMarkingType     lane_marking_type;
        LaneMarkingColor    color;
        LaneMarkingMaterial material;
        int sequence;
        double width;
        double length;           ///< Units:meters
        std::vector<Point3D> geometry;
    };
    
    /**
    
     * Lane information
     
    */ 
    struct Lane
    {
        int laneid;
        int linkid;
        int sequence;///< The lane in the Link's sequence number,
                     ///< Along the driving direction,the rightmost lane sequence =1
        LaneType lane_type;
        int forming_destory; ///< 0, normal lane; 1, Forming lane; 2, Destory lane
        int lane_status;
        Direction travel_direction;
        
        int min_speed_limit;     ///< Units:km/h
        int max_speed_limit;     ///< Units:km/h
        //Units: meters
        double width;            ///< Units:meters
        double length;           ///< Units:meters
        int vehiche_type;
        std::vector<LaneMarking*> marking_list;
        std::vector<Point3D> geometry; ///< center line geometry
    };

    /**
    
     * Connection between lanes
     
    */ 
    struct LaneTopology
    {
        int laneid_from;
        int linkid_from;
        int laneid_to;
        int linkid_to;
    };
    
    /**
    
     * Object main type
     
    */ 
    enum ObjectType
    {
        OBJECT_TYPE_POLE     = 1,
        OBJECT_TYPE_LinkMARK = 2,
        OBJECT_TYPE_SIGN     = 3,
        OBJECT_TYPE_BUILDING = 4,
        OBJECT_TYPE_GANTRY   = 5
    };
    
    /**
    
     * Object Sub Type
     
    */ 
    enum ObjectSubType
    {
        OST_NO_SUB_TYPE        = 0,
        OST_ROADMARK_ARROW     = 1,
        OST_ROADMARK_SHADEAREA = 2,
        OST_ROADMARK_CHARACTER = 3
    };
    
    /**
    
     * Object information
     
    */ 
    struct Object
    {
        int       id;
        ObjectType    type;
        ObjectSubType subtype;
        double center_x;     ///< Object center x
        double center_y;     ///< Object center y
        double center_z;     ///< Object center z
        float height;        ///< Bounding box height
        float width;         ///< Bounding box width
        float length;        ///< Bounding box length
        float heading;       ///< The angle between width's direction and gauss X's direction
        bool b_has_gemotry;  ///< Is the Object have the boundary geometry
        std::vector<Point3D> geometry;
        std::set<int> laneid;
    };

    /**
    
     * GPS Coord
     
    */ 
    struct GPS_Coord
    {
        double lon;

        double lat;

        double height;

        double heading;
    };
}

#endif//HAD_FEATURE_DEFINE_H
