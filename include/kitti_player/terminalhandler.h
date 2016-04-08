#ifndef _KITTI_TERMINAL_HANDLER_H_
#define _KITTI_TERMINAL_HANDLER_H_

#if !defined(_MSC_VER)
  #include <sys/select.h>
  #include <termios.h>
  #include <unistd.h>
#endif

class TerminalHandler
{
public:
	TerminalHandler();
	~TerminalHandler();
	int readCharFromStdin();

private:
	void restoreTerminal();
	void setupTerminal();

    bool    terminal_modified_;
#if defined(_MSC_VER)
    HANDLE input_handle;
    DWORD stdin_set;
#else
    termios orig_flags_;
    fd_set  stdin_fdset_;
#endif
    int     maxfd_;

};

#endif // _KITTI_TERMINAL_HANDLER_H_
