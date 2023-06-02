// UVIC CSC - Assignment 2
// Sebastian Klement
// Last modified 02.06.23
// "Simple" Raytracing Models


// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;
using namespace std;

const double EPSILON = 1e-6;

bool interTriangle(const Vector3d &ray_origin, const Vector3d &ray_direction, 
                    const Vector3d &v0, const Vector3d &v1, const Vector3d &v2, 
                    double &out_u, double &out_v, double &out_t)
{
    //based on the Möller-Trumbore algorithm 

    Vector3d e1 = v1 - v0;                      // Edge 1 from Triangle
    Vector3d e2 = v2 - v0;                      // Edge 2 from Triangle

    Vector3d h = ray_direction.cross(e2);       // Vector with 90° on ray_direction and e2
    double a = e1.dot(h);

    // Check Ray is parallel to triangle
    if (a > -EPSILON && a < EPSILON)
        return false;

    double f = 1.0 / a;                         //inverse of a and scaling factor for barycentric coordinates
    Vector3d s = ray_origin - v0;
    out_u = f * s.dot(h);

    // Check if intersection is outside of the triangle
    if (out_u < 0.0 || out_u > 1.0)
        return false;

    Vector3d q = s.cross(e1);
    out_v = f * ray_direction.dot(q);           //these are barycentric coordinates If they are between 0 and 1 the ray is inside the triangle
    
    // Check if intersection is outside of the triangle
    if (out_v < 0.0 || out_u + out_v > 1.0)
        return false;


    out_t = f * e2.dot(q);                      // distance from ray origin to intersection 

    // Check if there is a line intersection but not Ray intersection
    // Line is infinite to both ends and a ray has a starting point end only one infinite side
    if (out_t <= EPSILON)
        return false;

    // If all the other checks are false the ray has an insertion with the triangle
    return true;
}

bool intersect(const Vector3d &ray_origin, const Vector3d &ray_direction,
                const Vector3d &pgram_origin, const Vector3d &pgram_u, 
                const Vector3d &pgram_v, double &out_u, 
                double &out_v, double &out_t)
{
    // A parallelogram has 4 verticles "corners" 
    Vector3d v0 = pgram_origin;
    Vector3d v1 = pgram_origin + pgram_u;
    Vector3d v2 = pgram_origin + pgram_u + pgram_v;
    Vector3d v3 = pgram_origin + pgram_v;

    // Here you call the function for the first triangle - the "upper left" part of the parallelogram
    bool does_intersect1 = interTriangle(ray_origin, ray_direction, v0, v1, v2, out_u, out_v, out_t);
    
    // Here you call the function for the second triangle - the "down right" part of the parallelogram
    bool does_intersect2 = interTriangle(ray_origin, ray_direction, v0, v2, v3, out_u, out_v, out_t);


    // return both intersections
    return does_intersect1 || does_intersect2;
}

bool sphere_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& sphere_center, const double sphere_radius, double& t)
{
    Vector3d L = sphere_center - ray_origin;                        // Computes vector from ray_origin to the Sphere center
    double tca = L.dot(ray_direction);                              
    double d2 = L.dot(L) - tca * tca;                               // sqrt distance from center of sphere to ray
    double thc = std::sqrt(sphere_radius * sphere_radius - d2);
    t = tca - thc;

    if (d2 > sphere_radius * sphere_radius)
    {
        return false; // Ray doesnt hit the sphere
    }

    if (t < 0)  // intersection point is behind ray origin 
    {
        t = tca + thc;  // distance to second intersection point

        if (t < 0)
        {
            return false; // Ray starts inside the sphere
        }
    }

    return true; // Ray intersects the sphere

}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color # if displacement value is lower image is sharper so this needs to be bigger to correctly show the whole picture 
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask # needs to be as big as Matrix C. I think at least so big could be bigger maybe ?

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            // I use the image origin to calculate the displaces centers. Camera_origin doesnt work! (well it does but its not in the frame)
            const Vector3d displaced_centers = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray - rays should be parallel and NOT come from a single point
            const Vector3d ray_origin = displaced_centers;
            const Vector3d ray_direction = camera_view_direction.normalized();

            // Compute the normal vector of the parallelogram
            Vector3d pgram_normal = pgram_u.cross(pgram_v).normalized();

            double u, v, t;
            // implemented generic function to check if ray intersects with parallelogram because it is used 2 times in this whole code
            //bool does_intersect = interParallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v);
            //bool does_intersect = inter2triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v);
            bool does_intersect = intersect(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v, t);


            if (does_intersect)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                // Vector3d ray_intersection(0, 0, 0);
                //Vector3d ray_intersection = pgram_origin + u * pgram_u + v * pgram_v;
                Vector3d ray_intersection = ray_origin + ray_direction * t;

                // TODO: Compute normal at the intersection point
                // Vector3d ray_normal = ray_intersection.normalized();


                Vector3d ray_normal = pgram_normal; 
                // Simple diffuse model
                // C(i, j) = (light_position - ray_intersection).normalizewd().transpose() * ray_normal;
                // C(i, j) = (light_position - ray_intersection).normalized().transpose() * pgram_normal;
                Vector3d light_direction = (ray_intersection - light_position).normalized();
                C(i, j) = light_direction.dot(ray_normal);

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }

        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    Vector3d pgram_normal = pgram_u.cross(pgram_v).normalized();

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            //const Vector3d ray_origin = pixel_center;
            //const Vector3d ray_direction = camera_view_direction;


            // ray should come out of the camera 
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = pixel_center - camera_origin; 


            double u, v, t;
            //bool does_intersect = interParallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v);
            //bool does_intersect = inter2triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v);
            bool does_intersect = intersect(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v, t);

            // TODO: Check if the ray intersects with the parallelogram
            // the if statement is the same with the orthographic and the perspective projection 
            // only difference is the point where the rays are coming from. 
            // Single Point == Perspective.
            // Rays Parallel == orthographic
            if (does_intersect)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                // Vector3d ray_intersection(0, 0, 0);
                Vector3d ray_intersection = pgram_origin + u * pgram_u + v * pgram_v;

                // TODO: Compute normal at the intersection point
                // Vector3d ray_normal = ray_intersection.normalized();
                 


                Vector3d ray_normal = pgram_normal; 
                // Simple diffuse model
                // C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;
                // C(i, j) = (light_position - ray_intersection).normalized().transpose() * pgram_normal;
                Vector3d light_direction = (ray_intersection - light_position).normalized();
                    C(i, j) = light_direction.dot(ray_normal);

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd R = MatrixXd::Zero(800, 800); // Store the Red color
    MatrixXd G = MatrixXd::Zero(800, 800); // Store the Green color
    MatrixXd B = MatrixXd::Zero(800, 800); // Store the Blue color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(0, 1, 0); //tried different colors
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < R.cols(); ++i)
    {
        for (unsigned j = 0; j < R.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();

            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection
            double t = 0.0;
            bool intersects = sphere_intersection(ray_origin, ray_direction, sphere_center, sphere_radius, t);
        

            if (intersects)
            {
                // TODO: The ray hit the sphere, compute the exact intersection point
                // Vector3d ray_intersection(0, 0, 0);
                Vector3d ray_intersection = ray_origin + t * ray_direction;

                // TODO: Compute normal at the intersection point
                // Vector3d ray_normal = ray_intersection.normalized();
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();


                // TODO: Add shading parameter here
                // const double diffuse = (light_position - ray_intersection).normalized().dot(ray_normal);
                // const double specular = (light_position - ray_intersection).normalized().dot(ray_normal);

                const Vector3d light_direction = (light_position - ray_intersection).normalized();
                const double diffuse = std::max(light_direction.dot(ray_normal), 0.0);
                const double specular = std::pow(std::max(light_direction.dot(ray_direction), 0.0), specular_exponent);


                // Simple diffuse model
                // C(i, j) = ambient + diffuse + specular;
                Vector3d final_color = ambient * diffuse_color + diffuse * diffuse_color + specular * specular_color;
                final_color = final_color.cwiseMax(0.0).cwiseMin(1.0); // Clamp the colors between 0 and 1


                // Clamp to zero
                // C(i, j) = std::max(C(i, j), 0.);
                R(i, j) = final_color.x();
                G(i,j) = final_color.y();
                B(i,j) = final_color.z();

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    //raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
