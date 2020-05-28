//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

void Renderer::startRenderer(){
    terminated = false;
	num_thread = std::thread::hardware_concurrency();
	printf("Numer %d has been created in the Thread Pool\n", num_thread);
    m_threads.clear();
	for (int i = 0; i < num_thread; i++)
	{
		m_threads.push_back(std::thread(
			[this]{
				while (true) {
					Task t;
					{
						std::unique_lock<std::mutex> lock(this->m_task_mutex);
						// if terminated the pool or has tasks in queue, then singal all the waited thread
						cv.wait(lock, [this]{return !this->m_tasks.empty() || this->terminated;});
						if (this->terminated&& this->m_tasks.empty()) return; // if the thread is terminated, then return the thread for joinable 
						t= std::move(this->m_tasks.front());
						m_tasks.pop();
					}
					t(); // task execution
				}
			}
		));
	}
}

void Renderer::releaseRenderer(){
    terminated = true;
	cv.notify_all();
	for (auto& t : m_threads) {
		if (t.joinable()) {
			t.join();
		}
		else {
			// printf("Error occurred: thread %lld is not joinable\n", &t);
		}
	}
}


void Renderer::addTask(std::function<void()> task)
{
	{
		std::unique_lock<std::mutex> lock(m_task_mutex);
		m_tasks.push(task);
	}
	cv.notify_one();
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;
    int spp = 64;
    accum = 0;

    startRenderer();
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            this->addTask(std::bind(
                &Renderer::RenderSinglePixel, this,
                &framebuffer, &scene, m++, spp,
                i, j, scale, imageAspectRatio, eye_pos));
        }
    }
    releaseRenderer();
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

void Renderer::RenderSinglePixel(std::vector<Vector3f>* framebuffer, const Scene* scene, int m, int spp,
        int i, int j, float scale, float imageAspectRatio, Vector3f eye_pos)
{
    float x = (2 * (i + 0.5) / (float)scene->width - 1) * imageAspectRatio * scale;
    float y = (1 - 2 * (j + 0.5) / (float)scene->height) * scale;
    Vector3f dir = normalize(Vector3f(-x, y, 1));

    for (int k = 0; k < spp; k++){
        framebuffer->at(m) += scene->castRay(Ray(eye_pos, dir), 0) / spp;  
    }

    std::unique_lock<std::mutex> lock(this->m_accum_mutex);
    accum++;
    UpdateProgress( accum / (float)scene->height/(float)scene->width);
}
