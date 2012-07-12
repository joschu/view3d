
/* Xlib include files */
#include <X11/Xlib.h>


/* Standard C include file */
#include <stdio.h>
#include <stdlib.h>

int main() {
  Display *display;
  int screen_num;
  unsigned int display_width, display_height;
  char* display_name = NULL;
  char* progname = "progname";

  if ( (display=XOpenDisplay(display_name)) == NULL )

    {
      (void) fprintf( stderr, "%s: cannot connect to X server %s\n",
		      progname, XDisplayName(display_name));
      exit( -1 );
    }


  screen_num = DefaultScreen(display);
  display_width = DisplayWidth(display, screen_num);
  display_height = DisplayHeight(display, screen_num);
  printf("height: %i. width: %i\n", display_width, display_height);
  XCloseDisplay(display);

}
