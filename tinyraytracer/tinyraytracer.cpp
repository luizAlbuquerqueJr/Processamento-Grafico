#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"

struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
    float intensityRGB;
};

struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}
    // ve se o obj está na cena ou nao
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

// raio incidente e a normal
Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }

    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y)>1e-3)  {
        float d = -(orig.y +4)/dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir*d;
        if (d>0 && fabs(pt.x)<10 && pt.z<0 && pt.z>-1000 && d<spheres_dist) {
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0,1,0);
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(1, 0, .3) : Vec3f(1, 1, 1);
        }
    }
    return std::min(spheres_dist, checkerboard_dist)<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth=0) {
    Vec3f point, N;
    Material material;
    
    // momento de parada 
    if (depth>10 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        return Vec3f(0.0, 0.0, 0.0); // background color
    }
    // std::cout << ":Point"<< point<<"\n" ;

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    // std::cout << "direcao: " << reflect_orig << '\n';
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights) {
    const int   width    = 1000;
    const int   height   = 768;
    const float fov      = M_PI/2.;// campo de visao
    std::cout << "i: " << -height/(2.*tan(fov/2.)) << '\n';
    std::cout << Vec3f(0,0, -height/(2.*tan(fov/2.))).normalize()<<'\n';
    std::cout << Vec3f(1,1, -height/(2.*tan(fov/2.))).normalize()<<'\n';
    // 0.57735026919

    Vec3f posCamera = Vec3f(0,0,0);
    Vec3f direcao =Vec3f(10, 0,-10).normalize();
    Vec3f direcaoPadraoCamera =Vec3f(posCamera.x - posCamera.x, posCamera.y - posCamera.y,direcao.z - posCamera.z);

    float cosTeta = (direcao.x * direcaoPadraoCamera.x) + (direcao.y * direcaoPadraoCamera.y) + (direcao.z * direcaoPadraoCamera.z);
    cosTeta /= (pow(pow(direcao.x,2) + pow(direcao.y,2) + pow(direcao.z,2),0.5)* pow(pow(direcaoPadraoCamera.x,2) + pow(direcaoPadraoCamera.y,2) + pow(direcaoPadraoCamera.z,2),0.5));
    float tgTeta = pow(1-pow(cosTeta,2),0.5)/cosTeta;
    
    float distancia = 10;
    std::vector<Vec3f> framebuffer(width*height);
    std::cout << "cosTeta: " << cosTeta << '\n';
    std::cout << "tgTeta: " << tgTeta << '\n';
    float incrementoX = 1*direcao.z;
    
    std::cout << "pos camera: "<< Vec3f(0,0,0).normalize()<<"\n";
    
       
    // imprime uma string e o resultado da soma entre as variáveis a e b
       #pragma omp parallel for
    for (size_t j = 0; j<height; j++) { // actual rendering loop
        for (size_t i = 0; i<width; i++) {
            // std::cout << "i: " << i << '\n';
            
            // float dir_x =  (i + 0.5)+ 160*direcao.x  -  width/2.;
            // float dir_y = -(j + 0.5)+30*direcao.y + height/2.;    // this flips the image at the same time
            float dir_x =  (i + 0.5)  -  width/2.;
            float dir_y = -(j + 0.5) + height/2.;    // this flips the image at the same time
            // float dir_z = -height/(2.*tan(fov/2.)); //gy
            
            // dir_x += 500. ;
            // dir_y += 10 ;
            // dir_y += 5*1000/10 ;
            float dir_z = -width/(2.*tan(fov/2.)); //gy
            // float distancia = 10;
            // float dir_z = distancia*tan(fov/2.);
            
            // std::cout << "x,y,z: " << dir_x << "  "<< dir_y << "  " <<dir_z << '\n';
            // std::cout << "direcao" << Vec3f(dir_x, dir_y, dir_z) << '\n';
            framebuffer[i+j*width] = cast_ray(posCamera, Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out123.ppm",std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main() {
    
    Material      ivoryRed(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.9, 0.1, 0.1),   50.);
    Material      ivoryGreen(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.1, 0.9, 0.1),   50.);
    Material      ivoryBlue(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.1, 0.1, 0.9),   50.);
    
    // Material      ivory1(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0, 0, 1),   50.);
    Material      glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);
    Material red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(1, 0.1, 0.1),   0.);
    // // Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);

    std::vector<Sphere> spheres;
    // spheres.push_back(Sphere(Vec3f(0,    -2,   -6), 1,      glass));




    // spheres.push_back(Sphere(Vec3f(-3,    0,   -16), 1,      ivory));
    spheres.push_back(Sphere(Vec3f(-10*0.57735026919,    0,   -10), 1,      ivoryRed));
    spheres.push_back(Sphere(Vec3f(-10,    0,   -10), 1,      ivoryRed));
    spheres.push_back(Sphere(Vec3f(10,    0,   -5), 1,      ivoryRed));
    // spheres.push_back(Sphere(Vec3f(-5,    0,   -15), 1,      ivoryRed));
    spheres.push_back(Sphere(Vec3f(0,    5,   -10), 1,      ivoryGreen));
    spheres.push_back(Sphere(Vec3f(10,    0,   -10), 1,      ivoryBlue));
    spheres.push_back(Sphere(Vec3f(10*0.57735026919,    0,   -10), 1,      ivoryBlue));
    spheres.push_back(Sphere(Vec3f(0,    -2, -6), 1,      red_rubber));
    // spheres.push_back(Sphere(Vec3f(3,    3,   -50), 1,      ivoryGreen));
    
    // (x,y,z)
    // ---------->x
    //y
    //^
    //|
    //|
    //|

    ///z entrando

    // spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2,      glass));
    // spheres.push_back(Sphere(Vec3f( 1.5, -0.5, -18), 3, red_rubber));
    // spheres.push_back(Sphere(Vec3f( 7,    5,   -18), 3,     mirror));

    std::vector<Light>  lights;
    // lights.push_back(Light(Vec3f(0, 0,  20), 5));
    lights.push_back(Light(Vec3f( 0, 50, 0), 3));
    // lights.push_back(Light(Vec3f( 0, 20, -5), 6));
    // lights.push_back(Light(Vec3f( -10, 0, 0), 1));
    // lights.push_back(Light(Vec3f( 10, 0, 0), 1));
    // lights.push_back(Light(Vec3f( 10, -5, 0), 3));
    // lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));

    render(spheres, lights);

    return 0;
}

