/*
 * mpGTui.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: andre
 */

#include "mpGTui.h"

namespace rs_log_learn
{

mpGTui::mpGTui()
: testbutton("Test")
{
	testbutton.signal_clicked().connect(sigc::mem_fun(*this,
	            &mpGTui::on_testbutton_clicked));
	add(testbutton);
	show_all_children();
}

mpGTui::~mpGTui()
{
}



void mpGTui::spawnUI()
{
}

void mpGTui::on_testbutton_clicked()
{
	Gtk::MessageDialog dialog(*this, "This is a test");
	dialog.run();
}

} /* namespace rs_log_learn */
