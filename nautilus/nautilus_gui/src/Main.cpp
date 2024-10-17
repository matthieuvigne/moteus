#include <gtkmm/application.h>
#include <iostream>

#include "NautilusGUI.h"
#include "NautilusDriver.h"

int main (int argc, char *argv[])
{
    int baudrate = 4000000;
    std::string port = "/dev/spidev0.0";
    if (argc > 1)
    {
        port = argv[1];
        baudrate = std::stoi(argv[2]);
    }
    std::cout << "SPI communication started: " << port << ", clock: " << baudrate << "Hz." << std::endl;

    nautilus::Nautilus nautilus(port, baudrate);

    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    NautilusGUI window(&nautilus);
    app->run(window);
    return 0;
}


