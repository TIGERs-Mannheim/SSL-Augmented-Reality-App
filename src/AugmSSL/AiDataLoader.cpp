/*
 * AiDataLoader.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "AiDataLoader.h"
#include "augm_wrapper.pb.h"

#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::filesystem;

namespace tigers
{

AiDataLoader::~AiDataLoader() {};
AiDataLoaderFile::~AiDataLoaderFile() {};

AiDataLoaderFile::AiDataLoaderFile(string folder, int fps)
{
	this->fps = fps;
	path p(folder);
	try
	{
		if (exists(p) && is_directory(p))
		{
			typedef vector<path> vec;
			vec v;

			copy(directory_iterator(p), directory_iterator(), back_inserter(v));

			sort(v.begin(), v.end());

			for (vec::const_iterator it(v.begin()); it != v.end(); ++it)
			{
				tigers::AugmWrapperContainer container;
				loadAiDataFromFile(it->string(), &container);

				for(int i=0;i<container.wrapper_size();i++)
				{
					frames.push_back(container.wrapper(i));
				}
			}
			cout << "Loaded " << frames.size()
					<< " frames for " << fps << "fps, lasting "
					<< ((float) frames.size() / fps) << "s" << endl;
		}
		else
		{
			cerr << p << " is not a folder!" << endl;
		}
	}

	catch (const filesystem_error& ex)
	{
		cout << ex.what() << '\n';
	}
}



/**
 * increment frameCounter after getting current frame
 */
tigers::AugmWrapper AiDataLoaderFile::getNext()
{
	if(frames.empty())
	{
		return dummyWrapper;
	}
	if (frames.size() > frameCounter)
	{
		return frames.at(frameCounter++);
	}
	cerr << "no more AI data left..." << endl;
	return frames.at(frames.size()-1);
}

/**
 * increment frameCounter after getting current frame
 */
tigers::AugmWrapper AiDataLoaderFile::get(int i)
{
	if(frames.empty())
	{
		return dummyWrapper;
	}
	if (frames.size() > i)
	{
		frameCounter = i;
		return frames.at(frameCounter);
	}
	cerr << "no more AI data left..." << endl;
	return frames.at(frames.size()-1);
}

/**
 * get current frame, but do not increment frame counter
 */
tigers::AugmWrapper AiDataLoaderFile::getCurrent()
{
	if (frames.size() > frameCounter)
	{
		return frames.at(frameCounter);
	}
	return dummyWrapper;
}

int AiDataLoaderFile::loadAiDataFromFile(string fileName,
		tigers::AugmWrapperContainer * container)
{
	fstream input(fileName, ios::in | ios::binary);
	if (!input)
	{
		cout << fileName << ": File not found." << endl;
		return -2;
	}
	else if (!(*container).ParseFromIstream(&input))
	{
		cerr << "Failed to parse." << endl;
		return -1;
	}
	return 0;
}

void AiDataLoaderNetwork::onPacketReceived(char* data, size_t dataSize)
{
	tigers::AugmWrapper wrapper;
	try {
		bool parsed = wrapper.ParseFromArray(data, dataSize);
		if(!parsed)
			cerr << "Could not parse data (size: " << dataSize << ")" << endl;
		else {
			mutex.lock();
			latestWrapper = wrapper;
			mutex.unlock();
		}
	} catch(exception& e)
	{
		cerr << "Exception in AiDataLoaderNetwork::onPacketReceived: " << e.what() << endl;
	}
}

void AiDataLoaderNetwork::handle_receive_from(const boost::system::error_code& error,
		size_t bytes_recvd) {
	if (!error) {
		onPacketReceived(data_, bytes_recvd);
		receivedData = true;

		if(active)
		{
			socket.async_receive_from(boost::asio::buffer(data_, max_length),
				sender_endpoint,
				boost::bind(&AiDataLoaderNetwork::handle_receive_from, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
		}
	} else {
		std::cout << error.message() << std::endl;
	}
}

AiDataLoaderNetwork::AiDataLoaderNetwork(string netIp, string multicastIp, int port) :
		socket(io_service)
{
	// Create the socket so that multiple may be bound to the same address.
	boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address::from_string(netIp),
			port);
	socket.open(listen_endpoint.protocol());
	socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
	socket.bind(listen_endpoint);

	// Join the multicast group.
	socket.set_option(
			boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(multicastIp)));

	recv_thread = new boost::thread(boost::bind(&AiDataLoaderNetwork::run, this));
}

void AiDataLoaderNetwork::run()
{
	socket.async_receive_from(boost::asio::buffer(data_, max_length),
			sender_endpoint,
			boost::bind(&AiDataLoaderNetwork::handle_receive_from, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
	io_service.run();
}

AiDataLoaderNetwork::~AiDataLoaderNetwork()
{
	active = false;
	io_service.stop();
	if(recv_thread != NULL) {
		delete recv_thread;
		recv_thread = NULL;
	}
}

/**
 * increment frameCounter after getting current frame
 */
tigers::AugmWrapper AiDataLoaderNetwork::getNext()
{
	return getCurrent();
}

/**
 * increment frameCounter after getting current frame
 */
tigers::AugmWrapper AiDataLoaderNetwork::get(int i)
{
	return getCurrent();
}

/**
 * get current frame, but do not increment frame counter
 */
tigers::AugmWrapper AiDataLoaderNetwork::getCurrent()
{
	tigers::AugmWrapper wrapper;
	mutex.lock();
	wrapper = latestWrapper;
	mutex.unlock();
	return wrapper;
}

} /* namespace tigers */
