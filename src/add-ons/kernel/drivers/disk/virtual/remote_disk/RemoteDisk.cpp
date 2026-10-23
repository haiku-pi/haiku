/*
 * Copyright 2005-2007, Ingo Weinhold <bonefish@cs.tu-berlin.de>.
 * All rights reserved. Distributed under the terms of the MIT License.
 */

#include "RemoteDisk.h"

#include <new>

#include <endian.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <AutoDeleter.h>
#include <KernelExport.h>
#include <OS.h>

#include <kernel.h>		// for IS_USER_ADDRESS()


//#define TRACE_REMOTE_DISK
#ifdef TRACE_REMOTE_DISK
#	define TRACE(x) dprintf x
#else
#	define TRACE(x) do {} while (false)
#endif


static const int kMaxRequestResendCount = 5;
static const bigtime_t kReceiveTimeout = 800000LL;
static const bigtime_t kRequestTimeout = 5000000LL;

#if __BYTE_ORDER == __LITTLE_ENDIAN

static inline
uint64_t swap_uint64(uint64_t data)
{
	return ((data & 0xff) << 56)
		| ((data & 0xff00) << 40)
		| ((data & 0xff0000) << 24)
		| ((data & 0xff000000) << 8)
		| ((data >> 8) & 0xff000000)
		| ((data >> 24) & 0xff0000)
		| ((data >> 40) & 0xff00)
		| ((data >> 56) & 0xff);
}

#define host_to_net64(data)	swap_uint64(data)
#define net_to_host64(data)	swap_uint64(data)

#endif

#if __BYTE_ORDER == __BIG_ENDIAN
#define host_to_net64(data)	(data)
#define net_to_host64(data)	(data)
#endif

#undef htonll
#undef ntohll
#define htonll(data)	host_to_net64(data)
#define ntohll(data)	net_to_host64(data)


enum {
	BUFFER_SIZE	= 2048
};


// constructor
RemoteDisk::RemoteDisk()
	: fImageSize(0),
	  fRequestID(0),
	  fSocket(-1),
	  fPacket(NULL),
	  fPacketSize(0)
{
}


// destructor
RemoteDisk::~RemoteDisk()
{
	if (fSocket >= 0)
		close(fSocket);

	free(fPacket);
}


// Init
status_t
RemoteDisk::Init(uint32 serverAddress, uint16 serverPort, off_t imageSize)
{
	status_t error = _Init();
	if (error != B_OK)
		return error;

	fServerAddress.sin_family = AF_INET;
	fServerAddress.sin_port = htons(serverPort);
	fServerAddress.sin_addr.s_addr = htonl(serverAddress);
	fServerAddress.sin_len = sizeof(sockaddr_in);

	fImageSize = imageSize;

	return B_OK;
}


// FindAnyRemoteDisk
status_t
RemoteDisk::FindAnyRemoteDisk()
{
	status_t error = _Init();
	if (error != B_OK)
		return error;

	// prepare request
	remote_disk_header request;
	request.command = REMOTE_DISK_HELLO_REQUEST;

	// init server address to broadcast
	fServerAddress.sin_family = AF_INET;
	fServerAddress.sin_port = htons(REMOTE_DISK_SERVER_PORT);
	fServerAddress.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	fServerAddress.sin_len = sizeof(sockaddr_in);

	// set SO_BROADCAST on socket
	int soBroadcastValue = 1;
	if (setsockopt(fSocket, SOL_SOCKET, SO_BROADCAST, &soBroadcastValue,
			sizeof(soBroadcastValue)) < 0) {
		dprintf("RemoteDisk::Init(): Failed to set SO_BROADCAST on socket: "
			"%s\n", strerror(errno));
	}

	// send request
	sockaddr_in serverAddress;
	error = _SendRequest(&request, sizeof(request), REMOTE_DISK_HELLO_REPLY,
		&serverAddress);
	if (error != B_OK) {
		dprintf("RemoteDisk::FindAnyRemoteDisk(): Got no server reply: %s\n",
			strerror(error));
		return error;
	}
	remote_disk_header* reply = (remote_disk_header*)fPacket;

	// unset SO_BROADCAST on socket
	soBroadcastValue = 0;
	if (setsockopt(fSocket, SOL_SOCKET, SO_BROADCAST, &soBroadcastValue,
			sizeof(soBroadcastValue)) < 0) {
		dprintf("RemoteDisk::Init(): Failed to unset SO_BROADCAST on socket: "
			"%s\n", strerror(errno));
	}

	// init server address and size
	fServerAddress = serverAddress;
	fServerAddress.sin_port = reply->port;

	fImageSize = ntohll(reply->offset);

	return B_OK;
}


// ReadAt
ssize_t
RemoteDisk::ReadAt(off_t pos, void *_buffer, size_t bufferSize)
{
	if (fSocket < 0)
		return B_NO_INIT;

	uint8 *buffer = (uint8*)_buffer;
	if (!buffer || pos < 0)
		return B_BAD_VALUE;

	if (bufferSize == 0)
		return 0;

	// Check whether the current packet already contains the beginning of the
	// data to read.
	ssize_t bytesRead = _ReadFromPacket(pos, buffer, bufferSize);
	if (bytesRead < 0)
		return bytesRead;

	// More than a block: several requests at a time
	if (bufferSize > REMOTE_DISK_BLOCK_SIZE) {
		ssize_t pipelined = _ReadPipelined(pos, buffer, bufferSize);
		if (pipelined < 0)
			return bytesRead > 0 ? bytesRead : pipelined;
		return bytesRead + pipelined;
	}

	// If there still remains something to be read, we need to get it from the
	// server.
	status_t error = B_OK;
	while (bufferSize > 0) {
		// prepare request
		remote_disk_header request;
		request.offset = htonll(pos);
		uint32 toRead = min_c(bufferSize, REMOTE_DISK_BLOCK_SIZE);
		request.size = htons(toRead);
		request.command = REMOTE_DISK_READ_REQUEST;

		// send request
		error = _SendRequest(&request, sizeof(request), REMOTE_DISK_READ_REPLY);
		if (error != B_OK)
			break;

		// check for errors
		int16 packetSize = ntohs(((remote_disk_header*)fPacket)->size);
		if (packetSize < 0) {
			if (packetSize == REMOTE_DISK_IO_ERROR)
				error = B_IO_ERROR;
			else if (packetSize == REMOTE_DISK_BAD_REQUEST)
				error = B_BAD_VALUE;
			fPacketSize = 0;
			break;
		}

		// read from the packet
		size_t packetBytesRead = _ReadFromPacket(pos, buffer, bufferSize);
		if (packetBytesRead <= 0) {
			if (packetBytesRead < 0)
				error = packetBytesRead;
			break;
		}
		bytesRead += packetBytesRead;
	}

	// only return an error, when we were not able to read anything at all
	return (bytesRead == 0 ? error : bytesRead);
}


/*!	Reads \a bufferSize bytes at \a pos with up to kPipelineDepth requests
	of a block each in flight (the request ID tells the replies apart), each
	resent every kResendTimeout while its reply is overdue, and given up after
	as long as one request at a time could take (kMaxRequestResendCount times
	kRequestTimeout).
	One request at a time, a block per round trip, made the boot volume read
	at the network's latency, not its speed.
*/
ssize_t
RemoteDisk::_ReadPipelined(off_t pos, uint8* buffer, size_t bufferSize)
{
	static const int kPipelineDepth = 16;
	static const bigtime_t kResendTimeout = 500000;

	struct Slot {
		uint64		requestID;
		off_t		offset;
		uint32		size;
		bigtime_t	firstSentAt;
		bigtime_t	sentAt;
		bool		inFlight;
	} slots[kPipelineDepth];

	const uint32 blockCount
		= (bufferSize + REMOTE_DISK_BLOCK_SIZE - 1) / REMOTE_DISK_BLOCK_SIZE;
	uint32 nextBlock = 0;
	uint32 doneCount = 0;
	fPacketSize = 0;
		// fPacket serves as the receive buffer here

	auto send = [&](Slot& slot) -> status_t {
		remote_disk_header request;
		request.offset = htonll(slot.offset);
		request.size = htons(slot.size);
		request.command = REMOTE_DISK_READ_REQUEST;
		request.request_id = slot.requestID;
		request.port = fSocketAddress.sin_port;
		ssize_t sent;
		do {
			sent = sendto(fSocket, &request, sizeof(request), 0,
				(sockaddr*)&fServerAddress, sizeof(fServerAddress));
		} while (sent < 0 && errno == B_INTERRUPTED);
		if (sent != (ssize_t)sizeof(request))
			return sent < 0 ? errno : B_ERROR;
		slot.sentAt = system_time();
		return B_OK;
	};

	for (int i = 0; i < kPipelineDepth; i++)
		slots[i].inFlight = false;

	while (doneCount < blockCount) {
		// fill the free slots
		for (int i = 0; i < kPipelineDepth && nextBlock < blockCount; i++) {
			if (slots[i].inFlight)
				continue;
			Slot& slot = slots[i];
			slot.requestID = fRequestID++;
			slot.offset = pos + (off_t)nextBlock * REMOTE_DISK_BLOCK_SIZE;
			slot.size = min_c(REMOTE_DISK_BLOCK_SIZE,
				bufferSize - (size_t)nextBlock * REMOTE_DISK_BLOCK_SIZE);
			slot.inFlight = true;
			nextBlock++;
			status_t error = send(slot);
			if (error != B_OK)
				return error;
			slot.firstSentAt = slot.sentAt;
		}

		// a reply (the socket's receive timeout bounds the wait)
		ssize_t received = recvfrom(fSocket, fPacket, BUFFER_SIZE, 0, NULL,
			NULL);
		if (received < 0) {
			status_t error = errno;
			if (error != B_TIMED_OUT && error != B_WOULD_BLOCK
				&& error != B_INTERRUPTED) {
				return error;
			}
		} else if (received >= (ssize_t)sizeof(remote_disk_header)) {
			remote_disk_header* reply = (remote_disk_header*)fPacket;
			for (int i = 0; i < kPipelineDepth; i++) {
				Slot& slot = slots[i];
				if (!slot.inFlight || reply->request_id != slot.requestID
					|| reply->command != REMOTE_DISK_READ_REPLY) {
					continue;
				}
				int16 size = ntohs(reply->size);
				if (size < 0)
					return size == REMOTE_DISK_BAD_REQUEST ? B_BAD_VALUE : B_IO_ERROR;
				if ((size_t)size < slot.size
					|| received < (ssize_t)(sizeof(remote_disk_header) + slot.size)) {
					return B_IO_ERROR;
				}
				memcpy(buffer + (slot.offset - pos), reply->data, slot.size);
				slot.inFlight = false;
				doneCount++;
				break;
			}
		}

		// resend what is overdue
		const bigtime_t now = system_time();
		for (int i = 0; i < kPipelineDepth; i++) {
			Slot& slot = slots[i];
			if (!slot.inFlight || now - slot.sentAt < kResendTimeout)
				continue;
			if (now - slot.firstSentAt
					> kMaxRequestResendCount * kRequestTimeout) {
				dprintf("RemoteDisk: no reply for the block at %" B_PRIdOFF
					"\n", slot.offset);
				return B_TIMED_OUT;
			}
			status_t error = send(slot);
			if (error != B_OK)
				return error;
		}
	}

	return bufferSize;
}


/*!	Writes \a bufferSize bytes at \a pos the way _ReadPipelined() reads: up to
	kPipelineDepth requests of a block each in flight, matched to their replies
	by the request ID, each resent while its reply is overdue (a write at a given
	offset can be repeated). One request at a time wrote at the network's
	latency: ~2.9 MB/s on a Raspberry Pi 4's gigabit link.
	Returns B_NO_MEMORY without writing anything when it has no packet buffers;
	the caller then writes one block at a time.
*/
ssize_t
RemoteDisk::_WritePipelined(off_t pos, const uint8* buffer, size_t bufferSize)
{
	static const int kPipelineDepth = 16;
	static const bigtime_t kResendTimeout = 500000;
	static const size_t kPacketSize
		= sizeof(remote_disk_header) + REMOTE_DISK_BLOCK_SIZE;

	struct Slot {
		remote_disk_header* packet;
		size_t		packetSize;
		uint64		requestID;
		off_t		offset;
		uint32		size;
		bigtime_t	firstSentAt;
		bigtime_t	sentAt;
		bool		inFlight;
	} slots[kPipelineDepth];

	uint8* packets = (uint8*)malloc(kPipelineDepth * kPacketSize);
	if (packets == NULL)
		return B_NO_MEMORY;
	MemoryDeleter packetsDeleter(packets);

	const uint32 blockCount
		= (bufferSize + REMOTE_DISK_BLOCK_SIZE - 1) / REMOTE_DISK_BLOCK_SIZE;
	uint32 nextBlock = 0;
	uint32 doneCount = 0;
	fPacketSize = 0;
		// fPacket serves as the receive buffer here

	auto send = [&](Slot& slot) -> status_t {
		ssize_t sent;
		do {
			sent = sendto(fSocket, slot.packet, slot.packetSize, 0,
				(sockaddr*)&fServerAddress, sizeof(fServerAddress));
		} while (sent < 0 && errno == B_INTERRUPTED);
		if (sent != (ssize_t)slot.packetSize)
			return sent < 0 ? errno : B_ERROR;
		slot.sentAt = system_time();
		return B_OK;
	};

	for (int i = 0; i < kPipelineDepth; i++) {
		slots[i].packet = (remote_disk_header*)(packets + i * kPacketSize);
		slots[i].inFlight = false;
	}

	while (doneCount < blockCount) {
		// fill the free slots
		for (int i = 0; i < kPipelineDepth && nextBlock < blockCount; i++) {
			if (slots[i].inFlight)
				continue;
			Slot& slot = slots[i];
			slot.requestID = fRequestID++;
			slot.offset = pos + (off_t)nextBlock * REMOTE_DISK_BLOCK_SIZE;
			slot.size = min_c(REMOTE_DISK_BLOCK_SIZE,
				bufferSize - (size_t)nextBlock * REMOTE_DISK_BLOCK_SIZE);

			remote_disk_header* request = slot.packet;
			request->offset = htonll(slot.offset);
			request->size = htons(slot.size);
			request->command = REMOTE_DISK_WRITE_REQUEST;
			request->request_id = slot.requestID;
			request->port = fSocketAddress.sin_port;
			const uint8* data = buffer + (slot.offset - pos);
			if (IS_USER_ADDRESS(data)) {
				status_t error = user_memcpy(request->data, data, slot.size);
				if (error != B_OK)
					return error;
			} else
				memcpy(request->data, data, slot.size);
			slot.packetSize = sizeof(remote_disk_header) + slot.size;

			slot.inFlight = true;
			nextBlock++;
			status_t error = send(slot);
			if (error != B_OK)
				return error;
			slot.firstSentAt = slot.sentAt;
		}

		// a reply (the socket's receive timeout bounds the wait)
		ssize_t received = recvfrom(fSocket, fPacket, BUFFER_SIZE, 0, NULL,
			NULL);
		if (received < 0) {
			status_t error = errno;
			if (error != B_TIMED_OUT && error != B_WOULD_BLOCK
				&& error != B_INTERRUPTED) {
				return error;
			}
		} else if (received >= (ssize_t)sizeof(remote_disk_header)) {
			remote_disk_header* reply = (remote_disk_header*)fPacket;
			for (int i = 0; i < kPipelineDepth; i++) {
				Slot& slot = slots[i];
				if (!slot.inFlight || reply->request_id != slot.requestID
					|| reply->command != REMOTE_DISK_WRITE_REPLY) {
					continue;
				}
				int16 size = ntohs(reply->size);
				if (size < 0)
					return size == REMOTE_DISK_BAD_REQUEST ? B_BAD_VALUE : B_IO_ERROR;
				if ((size_t)size < slot.size)
					return B_IO_ERROR;
				slot.inFlight = false;
				doneCount++;
				break;
			}
		}

		// resend what is overdue
		const bigtime_t now = system_time();
		for (int i = 0; i < kPipelineDepth; i++) {
			Slot& slot = slots[i];
			if (!slot.inFlight || now - slot.sentAt < kResendTimeout)
				continue;
			if (now - slot.firstSentAt
					> kMaxRequestResendCount * kRequestTimeout) {
				dprintf("RemoteDisk: no reply for the write at %" B_PRIdOFF
					"\n", slot.offset);
				return B_TIMED_OUT;
			}
			status_t error = send(slot);
			if (error != B_OK)
				return error;
		}
	}

	return bufferSize;
}


// WriteAt
ssize_t
RemoteDisk::WriteAt(off_t pos, const void *_buffer, size_t bufferSize)
{
	if (fSocket < 0)
		return B_NO_INIT;

	const uint8 *buffer = (const uint8*)_buffer;
	if (!buffer || pos < 0)
		return B_BAD_VALUE;

	if (bufferSize == 0)
		return 0;

	if (bufferSize > REMOTE_DISK_BLOCK_SIZE) {
		ssize_t pipelined = _WritePipelined(pos, buffer, bufferSize);
		if (pipelined != B_NO_MEMORY)
			return pipelined;
	}

	status_t error = B_OK;
	size_t bytesWritten = 0;
	while (bufferSize > 0) {
		// prepare request
		remote_disk_header* request = (remote_disk_header*)fPacket;
		request->offset = htonll(pos);
		uint32 toWrite = min_c(bufferSize, REMOTE_DISK_BLOCK_SIZE);
		request->size = htons(toWrite);
		request->command = REMOTE_DISK_WRITE_REQUEST;

		// copy to packet buffer
		if (IS_USER_ADDRESS(buffer)) {
			status_t error = user_memcpy(request->data, buffer, toWrite);
			if (error != B_OK)
				return error;
		} else
			memcpy(request->data, buffer, toWrite);

		// send request
		size_t requestSize = request->data + toWrite - (uint8_t*)request;
		remote_disk_header reply;
		int32 replySize;
		error = _SendRequest(request, requestSize, REMOTE_DISK_WRITE_REPLY,
			NULL, &reply, sizeof(reply), &replySize);
		if (error != B_OK)
			break;

		// check for errors
		int16 packetSize = ntohs(reply.size);
		if (packetSize < 0) {
			if (packetSize == REMOTE_DISK_IO_ERROR)
				error = B_IO_ERROR;
			else if (packetSize == REMOTE_DISK_BAD_REQUEST)
				error = B_BAD_VALUE;
			break;
		}

		bytesWritten += toWrite;
		pos += toWrite;
		buffer += toWrite;
		bufferSize -= toWrite;
	}

	// only return an error, when we were not able to write anything at all
	return (bytesWritten == 0 ? error : bytesWritten);
}


// _Init
status_t
RemoteDisk::_Init()
{
	// open a control socket for playing with the stack
	fSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (fSocket < 0) {
		dprintf("RemoteDisk::Init(): Failed to open socket: %s\n",
			strerror(errno));
		return errno;
	}

	// bind socket
	fSocketAddress.sin_family = AF_INET;
	fSocketAddress.sin_port = 0;
	fSocketAddress.sin_addr.s_addr = INADDR_ANY;
	fSocketAddress.sin_len = sizeof(sockaddr_in);
	if (bind(fSocket, (sockaddr*)&fSocketAddress, sizeof(fSocketAddress)) < 0) {
		dprintf("RemoteDisk::Init(): Failed to bind socket: %s\n",
			strerror(errno));
		return errno;
	}

   // get the port
    socklen_t addrSize = sizeof(fSocketAddress);
    if (getsockname(fSocket, (sockaddr*)&fSocketAddress, &addrSize) < 0) {
		dprintf("RemoteDisk::Init(): Failed to get socket address: %s\n",
			strerror(errno));
        return errno;
    }

	// set receive timeout
	timeval timeout;
	timeout.tv_sec = time_t(kReceiveTimeout / 1000000LL);
	timeout.tv_usec = suseconds_t(kReceiveTimeout % 1000000LL);
	if (setsockopt(fSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))
			< 0) {
		dprintf("RemoteDisk::Init(): Failed to set socket receive timeout: "
			"%s\n", strerror(errno));
        return errno;
	}

	// allocate buffer
	fPacket = malloc(BUFFER_SIZE);
	if (!fPacket)
		return B_NO_MEMORY;

	return B_OK;
}


// _ReadFromPacket
ssize_t
RemoteDisk::_ReadFromPacket(off_t& pos, uint8*& buffer, size_t& bufferSize)
{
	if (fPacketSize == 0)
		return 0;

	// check whether the cached packet is indeed a read reply
	remote_disk_header* header = (remote_disk_header*)fPacket;
	if (header->command != REMOTE_DISK_READ_REPLY)
		return 0;

	uint64 packetOffset = ntohll(header->offset);
	uint32 packetSize = ntohs(header->size);
	if (packetOffset > (uint64)pos || packetOffset + packetSize <= (uint64)pos)
		return 0;

	// we have something to copy
	size_t toCopy = size_t(packetOffset + packetSize - (uint64)pos);
	if (toCopy > bufferSize)
		toCopy = bufferSize;

	if (IS_USER_ADDRESS(buffer)) {
		status_t error = user_memcpy(buffer,
			header->data + (pos - packetOffset), toCopy);
		if (error != B_OK)
			return error;
	} else
		memcpy(buffer, header->data + (pos - packetOffset), toCopy);

	pos += toCopy;
	buffer += toCopy;
	bufferSize -= toCopy;
	return toCopy;
}


// _SendRequest
status_t
RemoteDisk::_SendRequest(remote_disk_header* request, size_t size,
	uint8 expectedReply, sockaddr_in* peerAddress)
{
	return _SendRequest(request, size, expectedReply, peerAddress, fPacket,
		BUFFER_SIZE, &fPacketSize);
}


status_t
RemoteDisk::_SendRequest(remote_disk_header *request, size_t size,
	uint8 expectedReply, sockaddr_in* peerAddress, void* receiveBuffer,
	size_t receiveBufferSize, int32* _bytesReceived)
{
	request->request_id = fRequestID++;
	request->port = fSocketAddress.sin_port;

	// try sending the request kMaxRequestResendCount times at most
	for (int i = 0; i < kMaxRequestResendCount; i++) {
		// send request
		ssize_t bytesSent;
		do {
			bytesSent = sendto(fSocket, request, size, 0,
				(sockaddr*)&fServerAddress, sizeof(fServerAddress));
		} while (bytesSent < 0 && errno == B_INTERRUPTED);

		if (bytesSent < 0) {
			dprintf("RemoteDisk::_SendRequest(): failed to send packet: %s\n",
				strerror(errno));
			return errno;
		}
		if (bytesSent != (ssize_t)size) {
			dprintf("RemoteDisk::_SendRequest(): sent less bytes than "
				"desired\n");
			return B_ERROR;
		}

		// receive reply
		bigtime_t timeout = system_time() + kRequestTimeout;
		do {
			*_bytesReceived = 0;
			socklen_t addrSize = sizeof(sockaddr_in);
			ssize_t bytesReceived = recvfrom(fSocket, receiveBuffer,
				receiveBufferSize, 0, (sockaddr*)peerAddress,
				(peerAddress ? &addrSize : NULL));
			if (bytesReceived < 0) {
				status_t error = errno;
				if (error != B_TIMED_OUT && error != B_WOULD_BLOCK
						&& error != B_INTERRUPTED) {
					dprintf("RemoteDisk::_SendRequest(): recvfrom() failed: "
						"%s\n", strerror(error));
					return error;
				}
				continue;
			}

			// got something; check, if it is looks good
			if (bytesReceived >= (ssize_t)sizeof(remote_disk_header)) {
				remote_disk_header* reply = (remote_disk_header*)receiveBuffer;
				if (reply->request_id == request->request_id
					&& reply->command == expectedReply) {
					*_bytesReceived = bytesReceived;
					return B_OK;
				}
			}
		} while (timeout > system_time());
	}

	// no reply
	return B_TIMED_OUT;
}
