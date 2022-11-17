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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = Scene::intersect(ray);

    //not intersect with scene
    if (!inter.happened) {
        return Vector3f(0.0, 0.0, 0.0);
    }
    
    //intersect with light source
    Material* m = inter.m;
    if (m->hasEmission()) {
        return m->getEmission();
    }

    //intersect with object
    //calc direct light
    Vector3f L_dir;
    Vector3f emit, brdf, wl, wo, N, NN;
    float cos_theta, cos_theta_x, r, pdf;

    wo = ray.direction;
    Intersection inter_L;
    sampleLight(inter_L, pdf);
    wl = inter_L.coords - inter.coords;
    r = wl.norm();
    wl = normalize(wl);
    N = normalize(inter.normal);

    Ray ray_to_L(inter.coords, wl);
    Intersection inter_to_L = Scene::intersect(ray_to_L);
    if (inter_to_L.distance - r > -0.001)
    {
        emit = inter_L.emit;
        brdf = m->eval(wl, -wo, N);
        cos_theta = dotProduct(wl, N);
        cos_theta_x = dotProduct(-wl, inter_L.normal);

        L_dir = emit * brdf * cos_theta * cos_theta_x / r / r / pdf; 
    }
    
    //calc indirect light
    Vector3f L_indir = Vector3f(0.0f);
    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) {
        Vector3f wi = normalize(m->sample(wo, N));
        Ray ray_bouns(inter.coords, wi);
        Intersection inter_Obj = intersect(ray_bouns);
        
        if (inter_Obj.happened && !inter_Obj.m->hasEmission())
        {
            cos_theta = dotProduct(wi, N); 
            brdf = m->eval(wi, -wo, N);
            pdf = m->pdf(-wo, wi, N);

            L_indir = castRay(ray_bouns, depth + 1) * brdf * cos_theta / pdf / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}