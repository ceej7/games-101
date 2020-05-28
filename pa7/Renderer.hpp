//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <functional>

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void startRenderer();
    void releaseRenderer();
    void addTask(std::function<void()> task);
    void Render(const Scene& scene);
    void RenderSinglePixel(std::vector<Vector3f>* framebuffer, const Scene* scene, int m, int spp,
        int i, int j, float scale, float imageAspectRatio, Vector3f eye_pos
    );
private:
    using Task = std::function<void()>;
    std::vector<std::thread> m_threads;
	std::queue<Task> m_tasks;
	std::mutex m_task_mutex;
	std::condition_variable cv;
	int num_thread;
	bool terminated;
private:
    std::mutex m_accum_mutex;
    int accum=0;
};
