#ifndef GEOMETRY_H_
#define GEOMETRY_H_



namespace pcl_helpers
{
    typedef Eigen::Vector2f Vec2;
    typedef Eigen::Vector3f Vec3;
    typedef Eigen::Vector4f Vec4;
    typedef Eigen::Matrix3f Mat3x3;
    typedef Eigen::Matrix4f Mat4x4;
    typedef Eigen::Affine3f Transform;
    typedef Eigen::Quaternionf Quaternion;

    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2> > Vec2List;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Vec3List;
    typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4> > Vec4List;
    typedef std::vector<Mat3x3, Eigen::aligned_allocator<Mat3x3> > Mat3x3List;
    typedef std::vector<Mat4x4, Eigen::aligned_allocator<Mat4x4> > Mat4List;
    typedef std::vector<Transform, Eigen::aligned_allocator<Transform> > TransformList;
    typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionList;


    struct OBB
    {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Vec3 min;
            Vec3 max;
            Transform transform;
    };
}



#endif // GEOMETRY_H_ 
