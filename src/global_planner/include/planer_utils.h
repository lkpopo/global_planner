namespace global_planner
{
    struct Vec3
    {
        float x, y, z;

        Vec3() {};
        Vec3(float _x, float _y, float _z)
            : x(_x), y(_y), z(_z)
        {
        }
    };
    struct Pose
    {
        Vec3 position;
        Pose(Vec3 _pos) : position(_pos) {}
        Pose(float _x, float _y, float _z)
        {
            position.x = _x;
            position.y = _y;
            position.z = _z;
        }
    };

    struct Path
    {
        std::vector<Pose> poses;
    };

    struct GpsPoint
    {
        double lat;
        double lon;
        double alt;
    };
}
