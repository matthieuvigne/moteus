/// A GUI to test  and configure the STS3215 servos of the robot

#include <gtkmm/application.h>

#include "NautilusGUI.h"
#include "NautilusDriver.h"

int main (int argc, char *argv[])
{
    nautilus::Nautilus nautilus("/dev/spidev0.0");

    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    NautilusGUI window(&nautilus);
    app->run(window);
    return 0;
}


