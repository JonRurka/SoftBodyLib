#pragma once


#include "vk_types.h"

class VulkanBuffer;

class VulkanEngine {
public:
	bool isInitialized{ false };
	int frameNumber{ 0 };

	//initializes everything in the engine
	bool init();

	//shuts down the engine
	void cleanup();

	//run main loop
	void update();


	VulkanBuffer* getBuffer(size_t size, size_t stride);

	bool createComputeShaderModule(std::vector<char> content);
	bool createComputePipeline(std::vector<std::string> names);
	bool allocateBufferMemoryAndBind();
	bool allocateDescriptorSets();
	bool createCommandPoolAndBuffer();
	bool dispatchBindPipeline(uint32_t num);

	VkInstance instance;
	VkDebugUtilsMessengerEXT debug_messenger;
	VkPhysicalDevice chosenGPU;
	VkDevice device;

	VkQueue computeQueue;
	uint32_t computeQueueFamily;
private:

	bool init_vulkan();

	std::vector<VkDescriptorSetLayoutBinding> layoutBindings;
	VkDescriptorSetLayout setLayout;
	VkPipelineLayout pipelineLayout;
	VkPipeline* pipeline;
	size_t numPipelines;
	VkDeviceMemory memory;
	VkDescriptorPool descriptorPool;
	VkDescriptorSet descriptorSet;
	VkCommandPool commandPool;
	VkCommandBuffer commandBuffer;

	VkShaderModule shaderModule = VK_NULL_HANDLE;

	std::vector<VkBuffer> compute_buffers;
	std::vector<uint64_t> mem_offsets;

	uint64_t get_offset(int bindIndex) {
		uint64_t offset = 0;

		for (int i = 0; i < mem_offsets.size(); i++) {
			offset += mem_offsets[i];
		}

		return offset;
	}

	bool createPipelineLayout(VkDescriptorSetLayoutBinding newBinding);
	void addComputeBuffer(VkBuffer b)
	{
		compute_buffers.push_back(b);
	}

	uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

	friend class VulkanBuffer;
};

class VulkanBuffer {
	
public :
	VulkanBuffer(VulkanEngine* engineInst, size_t bSize, size_t stride);
	~VulkanBuffer();

	bool SetData(void* inData);
	bool GetData(void* outData);

	VulkanEngine* engine;

private:
	int bindIndex;
	size_t size;
	size_t stride;
	VkBuffer buff;

	bool createBinding();
	bool createBuffer();


	friend VulkanEngine;
};
