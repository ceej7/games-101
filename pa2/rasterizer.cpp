// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const std::array<Vector4f, 3>& _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    const Vector4f &p1=_v[0];
    const Vector4f &p2=_v[1];
    const Vector4f &p3=_v[2];
    float a = p2.x()-p1.x();
    float b = p3.x()-p1.x();
    float c = p2.y()-p1.y();
    float d = p3.y()-p1.y();
    float e = (float)x+0.5f-p1.x();
    float f = (float)y+0.5f-p1.y();
    if(a*d-b*c==0) return false;
    float coef = 1.0f/(a*d-b*c);
    float beta = (d*e-b*f)*coef;
    float gamma = (a*f-e*c)*coef;
    if(beta<0.0f||beta>1.0f) return false;
    if(gamma<0.0f||gamma>1.0f) return false;
    if(1.0f-beta-gamma<0.0f||1.0f-beta-gamma>1.0f) return false;
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            float w = vec.w();
            vec /= vec.w();
            vec<<vec.x(),vec.y(),vec.z(),w;
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    //step1 2d bounding box
    struct BoundingBox{
        float l=1000000.0f;
        float r=0.0f;
        float b=1000000.0f;
        float t=0.0f;
    };
    BoundingBox boundingBox={(float)width,0,(float)height,0};
    for(auto& vertex:v){
        boundingBox.l=std::min(vertex.x(), boundingBox.l);
        boundingBox.r=std::max(vertex.x(), boundingBox.r);
        boundingBox.b=std::min(vertex.y(), boundingBox.b);
        boundingBox.t=std::max(vertex.y(), boundingBox.t);
    }
    //step2 pixel traversal
    for(int i=(int)std::max(0.0f,boundingBox.l);i<(int)std::min((float)width,boundingBox.r+1.0f);i++){
        for(int j=(int)std::max(0.0f,boundingBox.b);j<(int)std::min((float)height,boundingBox.t+1.0f);j++){
            if(insideTriangle(i,j,v)){
                //step3 depth interpolation and test
                // If so, use the following code to get the interpolated z value.
                auto tp = computeBarycentric2D((float)i+0.5f, (float)j+0.5f, t.v);
                float alpha=std::get<0>(tp);float beta=std::get<1>(tp); float gamma=std::get<2>(tp);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());

                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                Eigen::Vector3f c_interpolated = alpha * t.color[0]*255.0f / v[0].w() + beta * t.color[1]*255.0f / v[1].w() + gamma * t.color[2]*255.0f/ v[2].w();
                c_interpolated*=w_reciprocal;

                int hit_place = (height-1-j)*width + i;
                if(z_interpolated<depth_buf[hit_place]){
                    depth_buf[hit_place] = z_interpolated;
                    set_pixel({(float)i,(float)j,0.0f},c_interpolated);
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
