/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Measures TCP throughput against a peer that sinks or sources data, like
	"socat -u TCP-LISTEN:5001 OPEN:/dev/null" or
	"socat -u TCP-LISTEN:5002 SYSTEM:'head -c 67108864 /dev/zero'".
*/


#include <OS.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>


static void
usage()
{
	fprintf(stderr, "usage: tcp_throughput send|receive <address> <port> "
		"[<megabytes to send>]\n");
	exit(1);
}


int
main(int argc, char** argv)
{
	if (argc < 4)
		usage();

	const bool sending = strcmp(argv[1], "send") == 0;
	if (!sending && strcmp(argv[1], "receive") != 0)
		usage();

	sockaddr_in address = {};
	address.sin_len = sizeof(address);
	address.sin_family = AF_INET;
	address.sin_port = htons(atoi(argv[3]));
	if (inet_pton(AF_INET, argv[2], &address.sin_addr) != 1)
		usage();
	const off_t total = (argc > 4 ? atoll(argv[4]) : 64) * 1024 * 1024;

	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0
		|| connect(sock, (sockaddr*)&address, sizeof(address)) != 0) {
		fprintf(stderr, "tcp_throughput: cannot connect to %s:%s: %s\n",
			argv[2], argv[3], strerror(errno));
		return 1;
	}

	static char buffer[64 * 1024];
	off_t transferred = 0;
	const bigtime_t start = system_time();
	while (!sending || transferred < total) {
		ssize_t bytes = sending
			? send(sock, buffer, sizeof(buffer), 0)
			: recv(sock, buffer, sizeof(buffer), 0);
		if (bytes < 0) {
			fprintf(stderr, "tcp_throughput: %s\n", strerror(errno));
			break;
		}
		if (bytes == 0)
			break;
		transferred += bytes;
	}
	const bigtime_t duration = system_time() - start;
	close(sock);

	printf("tcp_throughput: %s %lld bytes in %.2f s: %.2f MB/s (%.1f Mbit/s)\n",
		sending ? "sent" : "received", (long long)transferred, duration / 1e6,
		transferred / (double)duration, transferred * 8 / (double)duration);
	return 0;
}
