/*
MIT License
Copyright 2020 Jee W. Choi, Marat Dukhan, and Xing Liu
Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to use, 
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
Software, and to permit persons to whom the Software is furnished to do so, subject 
to the following conditions:
The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>
#ifndef _WIN32
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/stat.h>
#endif
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <CL/cl.h>
#include "timer.h"

#define PROCESS_BY_4_ELEMENTS

size_t array_size;

float *hostData = NULL;
float *hostData_ = NULL;

cl_device_id device = NULL;
cl_uint numDevices = 0;
cl_platform_id platform = NULL;
cl_command_queue commandQueue = NULL;
cl_context ctx = NULL;

cl_mem inputBuffer = NULL;
cl_mem inputBuffer_ = NULL;
cl_mem outputBuffer = NULL;
cl_mem outputBuffer_ = NULL;

cl_kernel kernel = NULL;
cl_program program = NULL;

/* ============================================================ */
/* Initialize the data structure on the host side */
void initHost() {
	::hostData = (cl_float*) malloc (sizeof (cl_float) * array_size);

	for (size_t i = 0; i < array_size; i++) {
		hostData[i] = float (i);
	}
}
/* ============================================================ */

/* ============================================================ */
/* Initialize the GPU platform */
void initCLPlatform() {
	cl_int status;
	cl_uint numberOfPlatforms;

	/* get the number of OpenCL platforms and their types */
	status = clGetPlatformIDs(1, &platform, &numberOfPlatforms);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetPlatformIDs failed with error: %d\n", status);
		exit(status);
	}

	/* get device IDs for the platforms found */
	status = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, 
													&numDevices);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetDeviceIDs failed with error %d\n", status);
		exit(status);
	}

	/* create an openCl context */
	::ctx = clCreateContext(0, 1, &::device, NULL, NULL, &status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateContext failed with error %d\n", status);
		exit(status);
	}

	/* create a command queue */
	::commandQueue = clCreateCommandQueue(::ctx, ::device, 
																				CL_QUEUE_PROFILING_ENABLE, &status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateCommandQueue failed with error %d\n", status);
		exit(status);
	}
}
/* ============================================================ */

/* ============================================================ */
/* Allocate device data */
void initCLBuffer() {
	cl_int status;

	/* input array */
	::inputBuffer = clCreateBuffer(::ctx, CL_MEM_READ_WRITE, sizeof(cl_float) 
																 * array_size, NULL, &status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateBuffer failed with error %d\n", status);
		exit(status);
	}

	/* output array */
	::outputBuffer = clCreateBuffer(::ctx, CL_MEM_READ_WRITE, sizeof(cl_float)
																	* array_size, NULL, &status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateBuffer failed with error %d\n", status);
		exit(status);
	}
}
/* ============================================================ */

/* ============================================================ */
/* Create and build the kernel */
void initCLKernel(int multiplyAdds) {
	cl_int status;

	/* Kernel source code */
	std::string source = 
	"__kernel void CLBench(__global float *input, __global float *output) {\n"
	"	uint id = get_global_id(0);\n"
#if defined(PROCESS_BY_16_ELEMENTS)
	"	float16 y = 0.0f;\n"
	"	float16 x = vload16(id, input);\n";
#elif defined(PROCESS_BY_8_ELEMENTS)
	"	float8 y = 0.0f;\n"
	"	float8 x = vload8(id, input);\n";
#elif defined(PROCESS_BY_4_ELEMENTS)
	"	float4 y = 0.0f;\n"
	"	float y_ = 0.0f;\n"
	"	float4 x = vload4(id, input);\n"
	"	float x_ = x.x * x.x;\n";
#elif defined(PROCESS_BY_2_ELEMENTS)
	"	float2 y = 0.0f;\n"
	"	float2 x = vload2(id, input);\n";
#else
	"	float y = 0.0f;\n"
	"	float x = input[id];\n";
#endif
	for (int i = 0; i < multiplyAdds; i++) {
#if defined(PROCESS_BY_16_ELEMENTS)
		source.append("	y = mad(x, x, y);\n");
#elif defined(PROCESS_BY_8_ELEMENTS)
		source.append("	y = mad(x, x, y);\n");
#elif defined(PROCESS_BY_4_ELEMENTS)
		source.append("	y = mad(x, x, y);\n");
		source.append("	y_ = mad(x_, x_, y_);\n");
#elif defined(PROCESS_BY_2_ELEMENTS)
		source.append("	y = mad(x, x, y);\n");
#else
		source.append("	y = mad(x, x, y);\n");
#endif
	}
#if defined(PROCESS_BY_16_ELEMENTS)
	source.append("	vstore16(y, id, output);\n");
#elif defined(PROCESS_BY_8_ELEMENTS)
	source.append("	vstore8(y, id, output);\n");
#elif defined(PROCESS_BY_4_ELEMENTS)
	source.append("	y.x = y.x + y_;\n");
	source.append("	vstore4(y, id, output);\n");
#elif defined(PROCESS_BY_2_ELEMENTS)
	source.append("	vstore2(y, id, output);\n");
#else
	source.append("	output[id] = y;\n");
#endif
	source.append("}\n\n");

	/* create a program using the source string */
	const char* sourceBuffer = source.c_str();
	::program = clCreateProgramWithSource(::ctx, 1, &sourceBuffer, NULL, 
																				&status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateProgramWithSource failed with error %d\n", 
						status);
		exit(status);
	}

	/* compile the program */
	status = clBuildProgram(::program, 0, NULL, 
													"-cl-fast-relaxed-math -cl-mad-enable", NULL, 
													NULL);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clBuildProgram failed with error %d %d\n", status, 
						CL_BUILD_PROGRAM_FAILURE);

		exit(status);
	}

	::kernel = clCreateKernel(::program, "CLBench", &status);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clCreateKernel failed with error %d\n", status);
		exit(status);
	}
}
/* ============================================================ */

/* ============================================================ */
/* Execute the kernel */
void runCL(int multiplyAdds) {
	cl_int status;
	cl_event writeEvent;
	cl_event computeEvent;

	/* Set input arguments to the kernel */
	status = clSetKernelArg(::kernel, 0, sizeof(cl_mem), &inputBuffer);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clSetKernelArg failed with error %d\n", status);
		exit(status);
	}

	status = clSetKernelArg(::kernel, 1, sizeof(cl_mem), &outputBuffer);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clSetKernelArg failed with error %d\n", status);
		exit(status);
	}


	/* Copy the data over to the device -- the command is only queued, not
		 executed */
	status = clEnqueueWriteBuffer(::commandQueue, inputBuffer, CL_FALSE, 0, 
																sizeof(cl_float) * array_size, hostData, 0, 
																NULL, &writeEvent);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clEnqueueWriteBuffer failed with error %d\n", status);
		exit(status);
	}



	/* Transfer data */
	/* clFinish is used instead of clFlush in order to make sure data copy
		 is completed before the time measurement begins */
	clFinish(::commandQueue);

#if defined(PROCESS_BY_16_ELEMENTS)
	size_t globalWorkSize[3] = {array_size / 16, 0, 0};
#elif defined(PROCESS_BY_8_ELEMENTS)
	size_t globalWorkSize[3] = {array_size / 8, 0, 0};
#elif defined(PROCESS_BY_4_ELEMENTS)
	size_t globalWorkSize[3] = {array_size / 4, 0, 0};
#elif defined(PROCESS_BY_2_ELEMENTS)
	size_t globalWorkSize[3] = {array_size / 2, 0, 0};
#else
	size_t globalWorkSize[3] = {array_size, 0, 0};
#endif
	status = clEnqueueNDRangeKernel(::commandQueue, ::kernel, 1, 0, 
																	globalWorkSize, NULL, 1, &writeEvent, 
																	&computeEvent);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clEnqueueNDRangeKernel failed with error %d %d\n", 
						status, CL_OUT_OF_RESOURCES);
		exit(status);
	}

	/* Create and initialize timer */
	struct stopwatch_t * timer = NULL;
	long double t_start, t_end;

	stopwatch_init ();
	timer = stopwatch_create ();

	/* Start timer */
	stopwatch_start (timer);

	/* Execute queued kernel */
	clFinish(::commandQueue);

	/* Stop timer */
	t_end = stopwatch_elapsed (timer);
	fprintf (stderr, "Execution time: %Lg secs\n", t_end);

	/* Measure various other times related to openCl execution */
	cl_ulong timeStart, timeEnd, timeQueued, timeSubmitted;
	status = clGetEventProfilingInfo(computeEvent, 
																	 CL_PROFILING_COMMAND_QUEUED, 
																	 sizeof (cl_ulong), &timeQueued, NULL);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetEventProfilingInfo failed with error %d\n", 
						status);
	}
	status = clGetEventProfilingInfo(computeEvent, 
																	 CL_PROFILING_COMMAND_SUBMIT, 
																	 sizeof (cl_ulong), &timeSubmitted, NULL);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetEventProfilingInfo failed with error %d\n", 
						status);
	}
	status = clGetEventProfilingInfo(computeEvent, CL_PROFILING_COMMAND_START,
																	 sizeof (cl_ulong), &timeStart, NULL);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetEventProfilingInfo failed with error %d\n", 
						status);
	}
	status = clGetEventProfilingInfo(computeEvent, CL_PROFILING_COMMAND_END, 
																	 sizeof (cl_ulong), &timeEnd, NULL);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clGetEventProfilingInfo failed with error %d\n", 
						status);
	}

	/* Print timing info based on OpenCL counters */
	printf("Computed (queued -> finish) in %5.3lf msecs\n", 
				 double(timeEnd - timeQueued) / 1.0e+6);
	printf("Computed (submit -> finish) in %5.3lf msecs\n", 
				 double(timeEnd - timeSubmitted) / 1.0e+6);
	printf("Computed (start -> finish) in %5.3lf msecs\n", 
				 double(timeEnd - timeStart) / 1.0e+6);
	printf("\tPerformance: %5.3lf GFLOPS\n", double(2.0 * (double(array_size) 
				 * 1.25 * double(multiplyAdds) / 1.0e+9) / (double(timeEnd - 
				 timeStart) / 1.0e+9)));
	printf("\tPerformance: %5.3lf GB/s\n", (2.0 * array_size * sizeof 
				 (cl_float)) / double((timeEnd - timeStart)));

	/* Print timing info based on custom timer */
	printf("Computed in %5.3Lg secs\n", t_end);
	printf("\tPerformance: %5.3lf GFLOPS\n", double(2.0 * (double(array_size) 
				 * 1.25 * double(multiplyAdds) / 1.0e+9) / t_end));
	printf("\tPerformance: %5.3Lg GB/s\n", ((2.0 * array_size * sizeof 
				 (cl_float))/1.0e+9) / t_end);

	/* Clean up */
	clReleaseEvent(writeEvent);
	clReleaseEvent(computeEvent);
}
/* ============================================================ */

/* ============================================================ */
void verify(int multiplyAdds) {
	cl_int status = CL_FALSE;
	cl_float* hostOutput = (cl_float*) malloc (sizeof (cl_float) * 
												 array_size);

	status = clEnqueueReadBuffer(::commandQueue, ::outputBuffer, CL_TRUE, 0, 
															 sizeof(cl_float) * array_size, hostOutput, 0,
															 NULL, 0);
	if (status != CL_SUCCESS) {
		fprintf(stderr, "clEnqueueReadBuffer failed with error %d\n", status);
		exit(status);
	}

	size_t countDiff = 0;
	for (size_t i = 0; i < array_size; i++) {
		if (i % 4 != 0) {
			if (fabsf(hostData[i] * hostData[i] * float(multiplyAdds) - hostOutput[i]) > 1.0e-4 * hostOutput[i]) {
				countDiff++;
			}
		} else {
			if (fabsf(hostData[i] * hostData[i] * float(multiplyAdds) + hostData[i]*hostData[i] * hostData[i]*hostData[i] * float(multiplyAdds) - hostOutput[i]) > 1.0e-4 * hostOutput[i]) 			{
				countDiff++;
			}
		}
	}

	if (countDiff != 0) {
		fprintf(stderr, "Output verification failed: num of mismatch: %zu\n", 
						countDiff);
	} else {
		fprintf(stderr, "Output verification success: num of mismatch: %zu\n", 
						countDiff);
	}

	free(hostOutput);
}
/* ============================================================ */


/* ============================================================ */
/* Free up allocated memory */
void cleanupCL() {
	free(hostData);

	clReleaseMemObject(::inputBuffer);
	clReleaseMemObject(::outputBuffer);
	clReleaseKernel(::kernel);
	clReleaseProgram(::program);
	clReleaseCommandQueue(::commandQueue);
	clReleaseContext(::ctx);
}
/* ============================================================ */


int main(int argc, char** argv) {

	/* number of multiply adds per word of data */
	int multiplyAdds;

	if(argc != 3) {
		fprintf (stderr, "usage: %s <array size (M)> <# MAD>\n", argv[0]);	
		exit (0);
	} else {
		array_size = atoi (argv[1]) * 1024 * 1024;
		multiplyAdds = atoi (argv[2]);
	}

	// initialize array
	initHost();

	// initialize the device
	initCLPlatform();

	// allocate device memory
	initCLBuffer();

	// create and build kernel
	initCLKernel(multiplyAdds);

	// run the kernel
	runCL(multiplyAdds);

	// verify results
	verify(multiplyAdds);

	// free memory
	cleanupCL();

	return 0;
}
