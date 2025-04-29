package main

/*
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>

int uart_open(const char *device) {
    int uart_filestream = open(device, O_RDWR | O_NOCTTY);
    if (uart_filestream == -1) {
        perror("Failed to open UART");
    }
    return uart_filestream;
}

void uart_configure(int uart_filestream) {
    struct termios options;
    tcgetattr(uart_filestream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_filestream, TCIFLUSH);
    tcsetattr(uart_filestream, TCSANOW, &options);
}

int uart_read(int uart_filestream, unsigned char *buffer, int buffer_size) {
    return read(uart_filestream, buffer, buffer_size);
}

int uart_read_with_select(int uart_filestream, unsigned char *buffer, int buffer_size, int timeout_sec) {
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(uart_filestream, &read_fds);

    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;

    int select_result = select(uart_filestream + 1, &read_fds, NULL, NULL, &timeout);
    if (select_result < 0) {
	    if (errno == EINTR) {
        	return 0;
        } else {
			perror("select() failed");
        	return -1;
		}
	} else if (select_result == 0) {
        // Timeout
        return 0;
    }

    return read(uart_filestream, buffer, buffer_size);
}

void uart_write(int uart_filestream, const char *data) {
    write(uart_filestream, data, strlen(data));
}

void uart_close(int uart_filestream) {
    close(uart_filestream);
}
*/
import "C"
import (
	"fmt"
	"os"
	"time"
	"unsafe"
)

func main() {
	startTime := time.Now()

	// Open UART
	device := "/dev/ttyAMA0" // Adjust this path as necessary
	uart := C.uart_open(C.CString(device))
	if uart == -1 {
		fmt.Println("Error: Unable to open UART.")
		os.Exit(1)
	}
	defer C.uart_close(uart)

	// Configure UART
	C.uart_configure(uart)

	// Start a goroutine for reading UART data
	done := make(chan bool)
	go readSerialData(uart, done, startTime)

	// Write data to UART every 2 seconds
	for {
		message := "Hello from Go\r\n"
		C.uart_write(uart, C.CString(message))
		//		fmt.Println("Sent message over Linux UART")
		time.Sleep(2 * time.Second)
	}
}

func readSerialData(uart C.int, done chan bool, startTime time.Time) {
	buffer := make([]byte, 256)

	for {
		// Read data from UART
		//n := C.uart_read(uart, (*C.uchar)(unsafe.Pointer(&buffer[0])), C.int(len(buffer)))
		n := C.uart_read_with_select(uart, (*C.uchar)(unsafe.Pointer(&buffer[0])), C.int(len(buffer)), 2)

		if n > 0 {
			elapsed := time.Since(startTime).Milliseconds()
			// Print received data
			fmt.Printf("%d ms :: Received (%d bytes) :: %s\n", int64(elapsed), int(n), string(buffer[:n]))
		} else if n < 0 {
			// Handle read error
			fmt.Printf("Error reading from Linux UART %d\n", int(n))
			done <- true
			return
		}

		// Non-blocking behavior: Timeout every 2 seconds
		//		time.Sleep(2 * time.Second)
	}
}
