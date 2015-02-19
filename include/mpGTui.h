/*
 * mpGTui.h
 *
 *  Created on: Feb 19, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_MPGTUI_H_
#define RS_LOG_LEARN_SRC_MPGTUI_H_

#include <gtkmm.h>

namespace rs_log_learn
{

class mpGTui : public Gtk::Window
{
public:
	mpGTui();
	virtual ~mpGTui();
	//static void activate(GtkApplication *app, gpointer user_data);

	void spawnUI();

protected:
	void on_testbutton_clicked();
	Gtk::Button testbutton;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPGTUI_H_ */
