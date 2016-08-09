/*
 * AiDataLoader.h
 *
 *  Created on: Jul 8, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AIDATALOADER_H_
#define AIDATALOADER_H_

#include "augm_wrapper.pb.h"
#include "augm_wrapper_container.pb.h"
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <mutex>

namespace tigers
{

class AiDataLoader
{
public:
	virtual ~AiDataLoader() = 0;
	virtual tigers::AugmWrapper getNext() = 0;
	virtual tigers::AugmWrapper get(int i) = 0;
	virtual tigers::AugmWrapper getCurrent() = 0;
};

class AiDataLoaderFile : public AiDataLoader
{
public:
	AiDataLoaderFile(std::string folder, int fps);
	virtual ~AiDataLoaderFile();
	tigers::AugmWrapper getNext();
	tigers::AugmWrapper get(int i);
	tigers::AugmWrapper getCurrent();

	static int loadAiDataFromFile(std::string fileName,
			tigers::AugmWrapperContainer * wrapper);

	int fps = 0;
private:
	AugmWrapper dummyWrapper;
	int frameCounter = 0;
	std::vector<tigers::AugmWrapper> frames;
};

class AiDataLoaderNetwork : public AiDataLoader
{
public:
	AiDataLoaderNetwork(std::string netIp, std::string multicastIp, int port);
	virtual ~AiDataLoaderNetwork();
	tigers::AugmWrapper getNext();
	tigers::AugmWrapper get(int i);
	tigers::AugmWrapper getCurrent();
	bool active = true;
	bool receivedData = false;

private:
	void onPacketReceived(char* data, size_t dataSize);
	void handle_receive_from(const boost::system::error_code& error,
			size_t bytes_recvd);
	void run();

	boost::thread* recv_thread = NULL;
	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint sender_endpoint;
	enum {
		max_length = 32768
	};
	char data_[max_length];
	AugmWrapper latestWrapper;
	std::mutex mutex;
};

} /* namespace tigers */

#endif /* AIDATALOADER_H_ */
