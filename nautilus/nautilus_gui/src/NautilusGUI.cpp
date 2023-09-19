#include "NautilusGUI.h"
#include "Commutation.h"
#include <iostream>


NautilusGUI::NautilusGUI(nautilus::Nautilus *nautilus):
    nautilus_(nautilus)
{
    Gtk::Grid *topGrid = new Gtk::Grid();

    topGrid->set_margin_top(10);
    topGrid->set_margin_bottom(5);
    topGrid->set_margin_left(10);
    topGrid->set_margin_right(10);
    topGrid->set_row_spacing(10);
    topGrid->set_column_spacing(10);
    this->add(*topGrid);

    Gtk::Label *header;
    header = new Gtk::Label("Register");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 0, 0, 1, 1);
    header = new Gtk::Label("Value");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 1, 0, 1, 1);
    header = new Gtk::Label("Write value");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 2, 0, 1, 1);
    Gtk::Separator *sep = new Gtk::Separator(Gtk::ORIENTATION_VERTICAL);
    topGrid->attach(*sep, 3, 0, 1, 1);
    sep->set_halign(Gtk::ALIGN_START);

    header = new Gtk::Label("Commands");
    header->set_hexpand(true);
    topGrid->attach(*header, 4, 0, 1, 1);

    topGrid->attach(scroll_, 0, 1, 5, 1);

    grid_.set_margin_bottom(10);
    grid_.set_row_spacing(10);
    grid_.set_column_spacing(10);
    grid_.set_size_request(800, 600);
    grid_.set_hexpand(true);
    grid_.set_vexpand(true);

    scroll_.set_size_request(800, 600);
    scroll_.add(grid_);

    // Add all registers

    header = new Gtk::Label("Auxiliary position");
    grid_.attach(*header, 0, 0, 1, 1);
    sizeGroups_[0]->add_widget(*header);
    grid_.attach(auxiliaryPosLabel_, 1, 0, 1, 1);
    sizeGroups_[1]->add_widget(auxiliaryPosLabel_);

    int i = 1;
    for (auto reg: nautilus::allRegisters)
    {
        Gtk::Label *name = new Gtk::Label(reg.name);
        grid_.attach(*name, 0, i, 1, 1);
        sizeGroups_[0]->add_widget(*name);

        registerValues_.push_back(Gtk::Label("0"));
        grid_.attach(registerValues_.back(), 1, i, 1, 1);
        sizeGroups_[1]->add_widget(registerValues_.back());

        if (reg.isWritable)
        {
            updateEntries_.push_back(std::pair<nautilus::NautilusRegister, Gtk::SpinButton>(reg, Gtk::SpinButton()));

            updateEntries_.back().second.set_numeric(true);
            if (reg.isFloat)
            {
                updateEntries_.back().second.set_range(-10000000, 10000000);
                updateEntries_.back().second.set_digits(3);
            }
            else
            {
                updateEntries_.back().second.set_range(-32766, 32766);
                updateEntries_.back().second.set_digits(0);
            }
            updateEntries_.back().second.set_width_chars(6);
            updateEntries_.back().second.set_increments(1, 10);
            updateEntries_.back().second.signal_value_changed().connect(sigc::bind(sigc::mem_fun(this, &NautilusGUI::writeRegister), updateEntries_.size() - 1));
            grid_.attach(updateEntries_.back().second, 2, i, 1, 1);
            sizeGroups_[2]->add_widget(updateEntries_.back().second);
        }
        i++;
    }

    sep = new Gtk::Separator(Gtk::ORIENTATION_VERTICAL);
    grid_.attach(*sep, 3, 0, 1, i);

    Gtk::Box *vBox = new Gtk::Box(Gtk::ORIENTATION_VERTICAL);
    vBox->set_spacing(10);
    vBox->set_valign(Gtk::ALIGN_START);
    vBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->set_hexpand(true);
    grid_.attach(*vBox, 4, 0, 1, i);
    vBox->pack_start(commStats_);


    Gtk::Box *hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Commutation current (A):");
    hBox->pack_start(*header);
    commutationCurrent_.set_range(0.1, 20.0);
    commutationCurrent_.set_increments(0.1, 1);
    commutationCurrent_.set_digits(2);
    commutationCurrent_.set_width_chars(4);
    hBox->pack_start(commutationCurrent_);
    vBox->pack_start(*hBox);

    commutationButton_ = Gtk::Button("Perform commutation");
    vBox->pack_start(commutationButton_);
    commutationButton_.signal_clicked().connect(sigc::mem_fun(this, &NautilusGUI::startCommutation));

    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::updateReadings), 100);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::checkAsyncStatus), 50);
    show_all();
}


bool NautilusGUI::updateReadings()
{
    for (unsigned int i = 0; i < registerValues_.size(); i++)
    {
        nautilus::NautilusReply rep = nautilus_->readRegister(nautilus::allRegisters[i]);
        if (rep.isValid)
            registerValues_[i].set_text(nautilus::allRegisters[i].isFloat ? std::to_string(rep.data) : std::to_string(reinterpret_cast<uint32_t &>(rep.data)));
        else
            registerValues_[i].set_text("Read fail!");
        if (i == 0)
            auxiliaryPosLabel_.set_text(rep.validEncoder ? std::to_string(rep.encoderPosition) : "Error");
    }
    double const sRate = nautilus_->nSuccess / static_cast<double>(nautilus_->nSuccess + nautilus_->nFailed) * 100.0;

    commStats_.set_text("Communication success rate:" + std::to_string(sRate) + " (" + std::to_string(nautilus_->nSuccess) + "/" + std::to_string(nautilus_->nFailed) + ")");
    return true;
}



void NautilusGUI::startCommutation()
{
    commutationButton_.set_sensitive(false);


    commutationDone_ = false;
    std::thread th = std::thread(performCommutation, nautilus_, commutationCurrent_.get_value(), &commutationDone_);
    th.detach();
}

bool NautilusGUI::checkAsyncStatus()
{
    using namespace std::chrono_literals;
    if (commutationDone_)
    {
        std::cout << "Commutation done..." << std::endl;
        commutationButton_.set_sensitive(true);
        commutationDone_ = false;
    }
    return true;
}


// void MainWindow::updateId(int const& servoNumber)
// {
//     // driver_->setTargetPosition(servoIds_[servoNumber], targetPositions_[servoNumber].get_value_as_int());
//     Gtk::Dialog dialog("ChangeId", *this);

//     dialog.add_button("Cancel", -1);
//     dialog.add_button("Change Id", 1);

//     Gtk::Box* box = dialog.get_content_area();
//     box->set_margin_start(10);
//     box->set_margin_end(10);
//     box->set_margin_left(10);
//     box->set_margin_right(10);
//     box->set_spacing(10);
//     Gtk::Label text("Set new id:");
//     box->pack_start(text);

//     Gtk::SpinButton button;
//     button.set_numeric(true);
//     button.set_range(0, 253);
//     button.set_increments(1, 10);
//     button.set_width_chars(4);
//     box->pack_start(button);
//     box->show_all();

//     int const responseId = dialog.run();
//     if (responseId > 0)
//     {
//         int const newId = button.get_value_as_int();
//         bool const wasIdChanged = driver_->setId(servoIds_[servoNumber], newId);
//         if (wasIdChanged)
//         {
//             servoIds_[servoNumber] = newId;
//             servoNames_[servoNumber].set_text(std::to_string(newId));
//         }
//     }

// }


// void MainWindow::resetPosition(int const& servoNumber)
// {
//     driver_->resetPositionAsCenter(servoIds_[servoNumber]);
// }


// void MainWindow::updateEnable(int const& servoNumber)
// {
//     if (torqueEnabled_[servoNumber].get_active())
//     {
//         targetPositions_[servoNumber].set_value(driver_->getCurrentPosition(servoIds_[servoNumber]));
//         targetVelocities_[servoNumber].set_value(0);
//         updateTargetPosition(servoNumber);
//         updateTargetVelocity(servoNumber);
//     }
//     driver_->disable(servoIds_[servoNumber], !torqueEnabled_[servoNumber].get_active());
// }


void NautilusGUI::writeRegister(int const& index)
{
    if (updateEntries_[index].first.isFloat)
        nautilus_->writeRegister(updateEntries_[index].first, static_cast<float>(updateEntries_[index].second.get_value()));
    else
        nautilus_->writeRegister(updateEntries_[index].first, static_cast<uint32_t>(updateEntries_[index].second.get_value()));
    updateReadings();
}


// void MainWindow::updateTargetVelocity(int const& servoNumber)
// {
//     if (torqueEnabled_[servoNumber].get_active())
//         driver_->setTargetVelocity(servoIds_[servoNumber], targetVelocities_[servoNumber].get_value_as_int());
// }


// void MainWindow::updateControlMode(int const& servoNumber)
// {
//     Glib::ustring const mode = controlModes_[servoNumber].get_active_text();
//     if (mode == "Position")
//     {
//         driver_->setMode(servoIds_[servoNumber], STS::Mode::POSITION);
//         usleep(5000);
//         targetPositions_[servoNumber].set_value(driver_->getCurrentPosition(servoIds_[servoNumber]));
//         updateTargetPosition(servoNumber);
//     }
//     else if (mode == "Velocity")
//     {
//         targetVelocities_[servoNumber].set_value(0);
//         updateTargetVelocity(servoNumber);
//         driver_->setMode(servoIds_[servoNumber], STS::Mode::VELOCITY);
//     }
//     else
//     {
//         targetPositions_[servoNumber].set_value(0);
//         updateTargetPosition(servoNumber);
//         driver_->setMode(servoIds_[servoNumber], STS::Mode::STEP);
//     }
// }