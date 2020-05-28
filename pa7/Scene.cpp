//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth, Intersection* tracedInter) const
{
    // max_depth
    if(depth>this->maxDepth) return 0;

    // intersection from the ray wo
    Intersection inter;
    if(tracedInter!=NULL)
        inter = *tracedInter;
    else
        inter = this->intersect(ray);
    if(!inter.happened) return backgroundColor;
    Vector3f pos = inter.coords;
    Vector3f normal = inter.normal;
    Object* obj = inter.obj;
    Material* m = inter.m;
    // ray wo property
    Vector3f wo = ray.direction;
    if(m->hasEmission()) return m->m_emission;

    // 1. Uniformly sample the light - visibility
    Vector3f L_dir = 0.0f;
    Intersection light_inter; float light_pdf;
    sampleLight(light_inter, light_pdf);
    Vector3f light_diff = light_inter.coords - pos;
    Vector3f light_dir = normalize( light_diff);
    float light_distance_double = dotProduct(light_diff,light_diff);
    // test light ray intersection
    auto light_inter_visibility = this->intersect(Ray(pos, light_dir));
    if(light_inter_visibility.obj==light_inter.obj)
    {
        L_dir=light_inter.emit*m->eval(wo, light_dir, normal)*std::max(dotProduct(light_dir, normal), 0.0f)
        * std::max(dotProduct(-light_dir, light_inter.normal), 0.0f)
        / std::max(light_distance_double, EPSILON) 
        / std::max(light_pdf, EPSILON);
    }


    // _ test Roulette with P_RR
    float ksi = get_random_float();
    if(ksi >RussianRoulette) return L_dir;

    // 2. sample the hemisphere and trace a ray
    Vector3f L_in = 0.0f;
    Vector3f wi = normalize(m->sample(wo, normal));
    float indir_pdf = m->pdf(wo, wi, normal);
    Intersection indir_inter = this->intersect(Ray(pos, wi));
    if(!indir_inter.happened||indir_inter.m->hasEmission()) return L_dir;
    L_in = castRay(Ray(pos, wi), depth+1, &indir_inter)*m->eval(wo, wi, normal)*std::max(dotProduct(wi, normal), 0.0f)
        / std::max(indir_pdf, EPSILON) 
        / std::max(RussianRoulette, EPSILON);

    return L_dir+L_in;

}