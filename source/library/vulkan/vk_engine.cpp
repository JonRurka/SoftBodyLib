#include "vk_engine.h"

#include "vk_types.h"
#include "vk_initializers.h"

#include "VkBootstrap.h"

constexpr bool bUseValidationLayers = true;
static int numBuffers = 0;
#define VK_CHECK(x)                                                 \
	do                                                              \
	{                                                               \
		VkResult err = x;                                           \
		if (err)                                                    \
		{                                                           \
			std::cout <<"Detected Vulkan error: " << err << std::endl; \
			abort();                                                \
		}                                                           \
	} while (0)



bool VulkanEngine::init()
{
	isInitialized = true;

	bool res;

	res = init_vulkan();


	isInitialized = res;
	return isInitialized;
}

//shuts down the engine
void VulkanEngine::cleanup()
{
	if (isInitialized) {
		
	}
}

//run main loop
void VulkanEngine::update()
{
	
}

bool VulkanEngine::init_vulkan()
{
	vkb::InstanceBuilder builder;

	auto inst_ret = builder.set_app_name("SoftBodyLib")
		.request_validation_layers(bUseValidationLayers)
		.use_default_debug_messenger()
		.require_api_version(1, 1, 0)
		.build();

	if (!inst_ret.has_value())
	{
		printf("ERROR: Failed to create Vulkan Instance.");
		return false;
	}

	vkb::Instance vkb_inst = inst_ret.value();

	instance = vkb_inst.instance;
	debug_messenger = vkb_inst.debug_messenger;

	//use vkbootstrap to select a gpu. 
	//We want a gpu that supports vulkan 1.2
	vkb::PhysicalDeviceSelector selector{ vkb_inst };
	vkb::PhysicalDevice physicalDevice = selector
		.set_minimum_version(1, 1)
		.select()
		.value();

	//create the final vulkan device

	vkb::DeviceBuilder deviceBuilder{ physicalDevice };
	
	vkb::Device vkbDevice = deviceBuilder.build().value();

	// Get the VkDevice handle used in the rest of a vulkan application
	device = vkbDevice.device;
	chosenGPU = physicalDevice.physical_device;

	// use vkbootstrap to get a queue
	computeQueue = vkbDevice.get_queue(vkb::QueueType::compute).value();
	computeQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::compute).value();

	

	return true;
}

VulkanBuffer* VulkanEngine::getBuffer(size_t size, size_t stride)
{
	VulkanBuffer* new_buff = new VulkanBuffer(this, size, stride);
	return new_buff;
}

bool VulkanEngine::createComputeShaderModule(std::vector<char> content)
{
	VkShaderModuleCreateInfo shaderModuleCreateInfo{};
	shaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	shaderModuleCreateInfo.pCode = reinterpret_cast<uint32_t*>(content.data());
	shaderModuleCreateInfo.codeSize = content.size();

	vkDestroyShaderModule(device, shaderModule, nullptr);
	if (vkCreateShaderModule(device, &shaderModuleCreateInfo, nullptr, &shaderModule) != VK_SUCCESS) {
		printf("failed to create shader module");
		return false;
	}

	return true;
}

bool VulkanEngine::createComputePipeline(std::vector<std::string> names)
{
	std::vector<VkComputePipelineCreateInfo> pipelineCreateInfos;

	for (int i = 0; i < names.size(); i++)
	{

		VkComputePipelineCreateInfo pipelineCreateInfo{};
		pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		pipelineCreateInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		pipelineCreateInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
		pipelineCreateInfo.stage.module = shaderModule;
		pipelineCreateInfo.stage.pName = names[i].c_str();
		pipelineCreateInfo.layout = pipelineLayout;
		pipelineCreateInfos.push_back(pipelineCreateInfo);
	}

	numPipelines = names.size();
	pipeline = new VkPipeline[numPipelines];
	if (vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, pipelineCreateInfos.data(), nullptr, pipeline) != VK_SUCCESS) {// TODO
		printf("failed to create compute pipeline");
		return false;
	}
	vkDestroyShaderModule(device, shaderModule, nullptr);

	return true;
}

bool VulkanEngine::createPipelineLayout(VkDescriptorSetLayoutBinding newBinding)
{
	layoutBindings.push_back(newBinding);

	VkDescriptorSetLayoutCreateInfo setLayoutCreateInfo{};
	setLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	setLayoutCreateInfo.bindingCount = static_cast<uint32_t>(layoutBindings.size());
	setLayoutCreateInfo.pBindings = layoutBindings.data();

	vkDestroyDescriptorSetLayout(device, setLayout, nullptr);
	if (vkCreateDescriptorSetLayout(device, &setLayoutCreateInfo, nullptr, &setLayout) != VK_SUCCESS) {
		printf("failed to create descriptor set layout!");
		return false;
	}

	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo{};
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.setLayoutCount = 1;
	pipelineLayoutCreateInfo.pSetLayouts = &setLayout;

	vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
	if (vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout)) {
		printf("failed to create pipeline layout");
		return false;
	}

	return true;
}

bool VulkanEngine::allocateBufferMemoryAndBind()
{
	VkDeviceSize requiredMemorySize = 0;
	uint32_t typeFilter = 0;

	for (const VkBuffer b : compute_buffers) 
	{
		VkMemoryRequirements bufferMemoryRequirements;
		vkGetBufferMemoryRequirements(device, b, &bufferMemoryRequirements);
		requiredMemorySize += bufferMemoryRequirements.size;
		typeFilter |= bufferMemoryRequirements.memoryTypeBits;
	}

	uint32_t memoryTypeIndex = findMemoryType(typeFilter, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	VkMemoryAllocateInfo allocateInfo = {};
	allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocateInfo.allocationSize = requiredMemorySize;
	allocateInfo.memoryTypeIndex = memoryTypeIndex;

	if (vkAllocateMemory(device, &allocateInfo, nullptr, &memory) != VK_SUCCESS) {
		printf("failed to allocate buffer memory");
		return false;
	}

	VkDeviceSize offset = 0;

	mem_offsets.clear();
	for (const VkBuffer b : compute_buffers)
	{
		VkMemoryRequirements bufferMemoryRequirements;
		vkGetBufferMemoryRequirements(device, b, &bufferMemoryRequirements);

		if (vkBindBufferMemory(device, b, memory, offset) != VK_SUCCESS) {
			throw std::runtime_error("failed to bind buffer memory");
		}

		mem_offsets.push_back(offset);
		offset += bufferMemoryRequirements.size;
	}

	return true;
}

bool VulkanEngine::allocateDescriptorSets()
{
	VkDescriptorPoolCreateInfo descriptorPoolCreateInfo{};
	descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolCreateInfo.maxSets = 1;

	VkDescriptorPoolSize poolSize{};
	poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	poolSize.descriptorCount = static_cast<uint32_t>(compute_buffers.size());

	descriptorPoolCreateInfo.poolSizeCount = 1;
	descriptorPoolCreateInfo.pPoolSizes = &poolSize;

	if (vkCreateDescriptorPool(device, &descriptorPoolCreateInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
		printf("failed to create descriptor pool");
		return false;
	}

	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{};
	descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	descriptorSetAllocateInfo.descriptorPool = descriptorPool;
	descriptorSetAllocateInfo.descriptorSetCount = 1;
	descriptorSetAllocateInfo.pSetLayouts = &setLayout;

	if (vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &descriptorSet) != VK_SUCCESS) {
		printf("failed to allocate descriptor sets");
		return false;
	}

	std::vector<VkWriteDescriptorSet> descriptorSetWrites(compute_buffers.size());
	std::vector<VkDescriptorBufferInfo> bufferInfos(compute_buffers.size());

	uint32_t i = 0;

	for (const VkBuffer& b : compute_buffers) {
		VkWriteDescriptorSet writeDescriptorSet{};
		writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet.dstSet = descriptorSet;
		writeDescriptorSet.dstBinding = i;
		writeDescriptorSet.dstArrayElement = 0;
		writeDescriptorSet.descriptorCount = 1;
		writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;

		VkDescriptorBufferInfo buffInfo{};
		buffInfo.buffer = b;
		buffInfo.offset = 0;
		buffInfo.range = VK_WHOLE_SIZE;
		bufferInfos[i] = buffInfo;

		writeDescriptorSet.pBufferInfo = &bufferInfos[i];
		descriptorSetWrites[i] = writeDescriptorSet;
		i++;
	}

	vkUpdateDescriptorSets(device, descriptorSetWrites.size(), descriptorSetWrites.data(), 0, nullptr);

	return true;
}

bool VulkanEngine::createCommandPoolAndBuffer()
{
	VkCommandPoolCreateInfo commandPoolCreateInfo{};
	commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
	commandPoolCreateInfo.queueFamilyIndex = computeQueueFamily;

	if (vkCreateCommandPool(device, &commandPoolCreateInfo, nullptr, &commandPool) != VK_SUCCESS) {
		printf("failed to create command pool");
		return false;
	}

	VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = commandPool;
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1;

	if (vkAllocateCommandBuffers(device, &commandBufferAllocateInfo, &commandBuffer) != VK_SUCCESS) {
		printf("failed to allocate command buffer");
		return false;
	}

	return true;
}

bool VulkanEngine::dispatchBindPipeline(uint32_t num)
{
	for (int i = 0; i < numPipelines; i++)
	{
		VkCommandBufferBeginInfo beginInfo{};
		beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(commandBuffer, &beginInfo);

		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline[i]);

		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, num, 1, 1);

		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
			printf("failed to end command buffer");
			return false;
		}
	}
	return true;
}

uint32_t VulkanEngine::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(chosenGPU, &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
			return i;
		}
	}

	throw std::runtime_error("failed to find suitable memory type!");
}

// ---------------------------------------------
// VulkanBuffer

VulkanBuffer::VulkanBuffer(VulkanEngine* engineInst, size_t bSize, size_t bStride)
{
	engine = engineInst;
	size = bSize;
	stride = bStride;
	createBinding();
	createBuffer();
}

VulkanBuffer::~VulkanBuffer()
{
	
}

bool VulkanBuffer::createBinding()
{
	bindIndex = numBuffers++;

	VkDescriptorSetLayoutBinding layoutBinding{};
	layoutBinding.binding = bindIndex;
	layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	layoutBinding.descriptorCount = 1;
	layoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;


	engine->createPipelineLayout(layoutBinding);

	return true;
}

bool VulkanBuffer::createBuffer()
{
	VkBufferCreateInfo bufferCreateInfo{};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.size = stride * size;
	bufferCreateInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	bufferCreateInfo.queueFamilyIndexCount = 1;
	bufferCreateInfo.pQueueFamilyIndices = &engine->computeQueueFamily;

	if (vkCreateBuffer(engine->device, &bufferCreateInfo, nullptr, &buff) != VK_SUCCESS) {
		printf("failed to create buffers");
		return false;
	}

	engine->addComputeBuffer(buff);

	return true;
}

bool VulkanBuffer::SetData(void* inData)
{
	float* data = nullptr;
	VkDeviceSize offset = engine->get_offset(bindIndex);

	VkMemoryRequirements bufferMemoryRequirements;
	vkGetBufferMemoryRequirements(engine->device, buff, &bufferMemoryRequirements);

	if (vkMapMemory(engine->device, engine->memory, offset, bufferMemoryRequirements.size, 0, reinterpret_cast<void**>(&data)) != VK_SUCCESS) {
		printf("failed to map device memory");
		return false;
	}

	memcpy(data, inData, size);

	vkUnmapMemory(engine->device, engine->memory);

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &engine->commandBuffer;
	vkQueueSubmit(engine->computeQueue, 1, &submitInfo, VK_NULL_HANDLE);
	// but we can simply wait for all the work to be done
	vkQueueWaitIdle(engine->computeQueue);

	return true;
}

bool VulkanBuffer::GetData(void* outData)
{
	float* data = nullptr;
	VkDeviceSize offset = engine->get_offset(bindIndex);

	VkMemoryRequirements bufferMemoryRequirements;
	vkGetBufferMemoryRequirements(engine->device, buff, &bufferMemoryRequirements);

	if (vkMapMemory(engine->device, engine->memory, offset, bufferMemoryRequirements.size, 0, reinterpret_cast<void**>(&data)) != VK_SUCCESS) {
		printf("failed to map device memory");
		return false;
	}

	memcpy(outData, data, size);

	vkUnmapMemory(engine->device, engine->memory);

	return true;
}


