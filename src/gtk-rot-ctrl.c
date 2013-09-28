/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2011  Alexandru Csete, OZ9AEC.

  Authors: Alexandru Csete <oz9aec@gmail.com>
           Charles Suprin  <hamaa1vs@gmail.com>

  Comments, questions and bugreports should be submitted via
  http://sourceforge.net/projects/gpredict/
  More details can be found at the project home page:

  http://gpredict.oz9aec.net/

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, visit http://www.fsf.org/
*/
/** \brief ROTOR control window.
 *  \ingroup widgets
 *
 * The master rotator control UI is implemented as a Gtk+ Widget in order
 * to allow multiple instances. The widget is created from the module
 * popup menu and each module can have several rotator control windows
 * attached to it. Note, however, that current implementation only
 * allows one rotor control window per module.
 * 
 */
#include <gtk/gtk.h>
#include <glib/gi18n.h>
#include <math.h>
#include "compat.h"
#include "sat-log.h"
#include "predict-tools.h"
#include "gpredict-utils.h"
#include "gtk-polar-plot.h"
#include "gtk-rot-knob.h"
#include "gtk-rot-ctrl.h"
#include "orbit-tools.h"
#include "sat-cfg.h"
#ifdef HAVE_CONFIG_H
#  include <build-config.h>
#endif

/* NETWORK */
//#include <sys/types.h>
#ifndef WIN32
#include <sys/socket.h>     /* socket(), connect(), send() */
#include <netinet/in.h>     /* struct sockaddr_in */
#include <arpa/inet.h>      /* htons() */
#include <netdb.h>          /* gethostbyname() */
#else
#include <winsock2.h>
#endif
/* END */

#define FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5


static void gtk_rot_ctrl_class_init (GtkRotCtrlClass *class);
static void gtk_rot_ctrl_init       (GtkRotCtrl      *list);
static void gtk_rot_ctrl_destroy    (GtkObject       *object);


static GtkWidget *create_az_widgets (GtkRotCtrl *ctrl);
static GtkWidget *create_el_widgets (GtkRotCtrl *ctrl);
static GtkWidget *create_target_widgets (GtkRotCtrl *ctrl);
static GtkWidget *create_conf_widgets (GtkRotCtrl *ctrl);
static GtkWidget *create_plot_widget (GtkRotCtrl *ctrl);

static void store_sats (gpointer key, gpointer value, gpointer user_data);

static void sat_selected_cb (GtkCellRendererToggle *cell_renderer, gchar *path_str, gpointer user_data);
static void sat_changed_minCommunication_cb(GtkCellRendererText *cellrenderertext, gchar *arg1, gchar *arg2, gpointer user_data);
static void track_toggle_cb (GtkToggleButton *button, gpointer data);
static void delay_changed_cb (GtkSpinButton *spin, gpointer data);
static void toler_changed_cb (GtkSpinButton *spin, gpointer data);
static void rot_selected_cb (GtkComboBox *box, gpointer data);
static void rot_locked_cb (GtkToggleButton *button, gpointer data);
static gboolean rot_ctrl_timeout_cb (gpointer data);
static void update_count_down (GtkRotCtrl *ctrl, gdouble t);
static void update_tracked_elem(GtkRotCtrl * ctrl);

static gboolean get_pos (GtkRotCtrl *ctrl, gdouble *az, gdouble *el);
static gboolean set_pos (GtkRotCtrl *ctrl, gdouble az, gdouble el);

static gboolean send_rotctld_command(GtkRotCtrl *ctrl, gchar *buff, gchar *buffout, gint sizeout);
static gboolean open_rotctld_socket (GtkRotCtrl *ctrl);
static gboolean close_rotctld_socket (gint *sock);

static gboolean have_conf (void);
static gint sat_name_compare (sat_t* a,sat_t*b);
static gint rot_name_compare (const gchar* a,const gchar *b);

static gboolean is_flipped_pass (pass_t * pass,rot_az_type_t type);
static inline void set_flipped_pass (GtkRotCtrl* ctrl);

static void append_elem_priority_queue(GtkRotCtrl* ctrl, int i);
static void remove_elem_priority_queue(GtkRotCtrl* ctrl, int i);
static void swap_elem_priority_queue(GtkRotCtrl* ctrl, int i, int j);
static int get_elem_index_priority_queue(GtkRotCtrl *ctrl, int i);

static void next_elem_to_track(GtkRotCtrl* ctrl);

static GtkVBoxClass *parent_class = NULL;

static GdkColor ColBlack = { 0, 0, 0, 0};
static GdkColor ColWhite = { 0, 0xFFFF, 0xFFFF, 0xFFFF};
static GdkColor ColRed =   { 0, 0xFFFF, 0, 0};
static GdkColor ColGreen = {0, 0, 0xFFFF, 0};


GType
        gtk_rot_ctrl_get_type ()
{
    static GType gtk_rot_ctrl_type = 0;

    if (!gtk_rot_ctrl_type) {

        static const GTypeInfo gtk_rot_ctrl_info = {
            sizeof (GtkRotCtrlClass),
            NULL,  /* base_init */
            NULL,  /* base_finalize */
            (GClassInitFunc) gtk_rot_ctrl_class_init,
            NULL,  /* class_finalize */
            NULL,  /* class_data */
            sizeof (GtkRotCtrl),
            5,     /* n_preallocs */
            (GInstanceInitFunc) gtk_rot_ctrl_init,
            NULL
        };

        gtk_rot_ctrl_type = g_type_register_static (GTK_TYPE_VBOX,
                                                    "GtkRotCtrl",
                                                    &gtk_rot_ctrl_info,
                                                    0);
    }

    return gtk_rot_ctrl_type;
}


static void
        gtk_rot_ctrl_class_init (GtkRotCtrlClass *class)
{
    //GObjectClass      *gobject_class;
    GtkObjectClass    *object_class;
    //GtkWidgetClass    *widget_class;
    //GtkContainerClass *container_class;

    //gobject_class   = G_OBJECT_CLASS (class);
    object_class    = (GtkObjectClass*) class;
    //widget_class    = (GtkWidgetClass*) class;
    //container_class = (GtkContainerClass*) class;

    parent_class = g_type_class_peek_parent (class);

    object_class->destroy = gtk_rot_ctrl_destroy;

}



static void
        gtk_rot_ctrl_init (GtkRotCtrl *ctrl)
{
    ctrl->checkSatsList = NULL;
    ctrl->sats = NULL;

    // Tiago's modification
    ctrl->target.numSatToTrack = 0;
    ctrl->target.priorityQueue = NULL;
    ctrl->target.sats = NULL;
    ctrl->target.minCommunication = NULL;
    ctrl->target.minElevation = 0.0f;

 //   ctrl->target = NULL;
 //   ctrl->pass = NULL;
    ctrl->qth = NULL;
    ctrl->plot = NULL;
    ctrl->sock = 0;

    ctrl->tracking = FALSE;
    g_static_mutex_init(&(ctrl->busy));
    ctrl->engaged = FALSE;
    ctrl->delay = 1000;
    ctrl->timerid = 0;
    ctrl->tolerance = 5.0;
    ctrl->errcnt = 0;
}

static void
        gtk_rot_ctrl_destroy (GtkObject *object)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (object);
    
    /* stop timer */
    if (ctrl->timerid > 0) 
        g_source_remove (ctrl->timerid);

    /* free configuration */
    if (ctrl->conf != NULL) {
        g_free (ctrl->conf->name);
        g_free (ctrl->conf->host);
        g_free (ctrl->conf);
        ctrl->conf = NULL;
    }
    
    /*close the socket if it is still open*/
    if (ctrl->sock!=0) {
        close_rotctld_socket(&(ctrl->sock));
    }

    (* GTK_OBJECT_CLASS (parent_class)->destroy) (object);
}



/** \brief Create a new rotor control widget.
 * \return A new rotor control window.
 * 
 */
GtkWidget *
        gtk_rot_ctrl_new (GtkSatModule *module)
{
    GtkWidget *widget;
    GtkWidget *table;
    guint num_sats;

    /* check that we have rot conf */
    if (!have_conf()) {
        return NULL;
    }
    
    widget = g_object_new (GTK_TYPE_ROT_CTRL, NULL);
    
    /* store satellites */
    g_hash_table_foreach (module->satellites, store_sats, widget);

    // Tiago's modification
 //   GTK_ROT_CTRL (widget)->target.targeting = SAT (g_slist_nth_data (GTK_ROT_CTRL (widget)->sats, 0));
 //   GTK_ROT_CTRL (widget)->target.numSatToTrack++;

    /* store current time (don't know if real or simulated) */
    GTK_ROT_CTRL (widget)->t = module->tmgCdnum;
    
    /* store QTH */
    GTK_ROT_CTRL (widget)->qth = module->qth;
     
    /* Init structure target */
    num_sats = g_slist_length (GTK_ROT_CTRL (widget)->sats);
    GTK_ROT_CTRL (widget)->target.sats = malloc(sizeof(int)*num_sats);
    GTK_ROT_CTRL (widget)->target.minCommunication = malloc(sizeof(float)*num_sats);
    GTK_ROT_CTRL (widget)->target.priorityQueue = malloc(sizeof(int)*num_sats);
    
    /* get next pass for target satellite */
   /* if (GTK_ROT_CTRL (widget)->target.targeting){
        if (GTK_ROT_CTRL (widget)->target.targeting->el > 0.0) {
            GTK_ROT_CTRL (widget)->target.pass =  get_current_pass (GTK_ROT_CTRL (widget)->target.targeting,
                                                            GTK_ROT_CTRL (widget)->qth,
                                                            0.0);
        }
        else {
            GTK_ROT_CTRL (widget)->target.pass =  get_next_pass (GTK_ROT_CTRL (widget)->target.targeting,
                                                         GTK_ROT_CTRL (widget)->qth,
                                                         3.0);
        }
    }*/
    
    /* initialise custom colors */
    gdk_rgb_find_color (gtk_widget_get_colormap (widget), &ColBlack);
    gdk_rgb_find_color (gtk_widget_get_colormap (widget), &ColWhite);
    gdk_rgb_find_color (gtk_widget_get_colormap (widget), &ColRed);
    gdk_rgb_find_color (gtk_widget_get_colormap (widget), &ColGreen);

    /* create contents */
    table = gtk_table_new (4, 4, FALSE);
    gtk_table_set_row_spacings (GTK_TABLE (table), 0);
    gtk_table_set_col_spacings (GTK_TABLE (table), 0);
    gtk_container_set_border_width (GTK_CONTAINER (table), 10);
    gtk_table_attach (GTK_TABLE (table), create_az_widgets (GTK_ROT_CTRL (widget)),
                      0, 1, 0, 1, GTK_FILL, GTK_SHRINK, 0, 0);
    gtk_table_attach (GTK_TABLE (table), create_el_widgets (GTK_ROT_CTRL (widget)),
                      1, 2, 0, 1, GTK_FILL, GTK_SHRINK, 0, 0);
    gtk_table_attach (GTK_TABLE (table), create_target_widgets (GTK_ROT_CTRL (widget)),
                      0, 4, 2, 4, GTK_FILL | GTK_EXPAND, GTK_FILL | GTK_EXPAND, 0, 0);
    gtk_table_attach (GTK_TABLE (table), create_conf_widgets (GTK_ROT_CTRL (widget)),
                      0, 3, 1, 2, GTK_FILL | GTK_EXPAND, GTK_SHRINK , 0, 0);
    gtk_table_attach (GTK_TABLE (table), create_plot_widget (GTK_ROT_CTRL (widget)),
                      3, 4, 0, 2, GTK_FILL | GTK_EXPAND, GTK_FILL | GTK_EXPAND, 0, 0);

    gtk_container_add (GTK_CONTAINER (widget), table);
    
    GTK_ROT_CTRL (widget)->timerid = g_timeout_add (GTK_ROT_CTRL (widget)->delay,
                                                    rot_ctrl_timeout_cb,
                                                    GTK_ROT_CTRL (widget));
    
    return widget;
}


/** \brief Update rotator control state.
 * \param ctrl Pointer to the GtkRotCtrl.
 * 
 * This function is called by the parent, i.e. GtkSatModule, indicating that
 * the satellite data has been updated. The function updates the internal state
 * of the controller and the rotator.
 */
void
        gtk_rot_ctrl_update   (GtkRotCtrl *ctrl, gdouble t)
{
    gchar *buff;
    
    ctrl->t = t;
    
    // Tiago's modification
    if (ctrl->target.targeting) {

        /* update target displays */
        buff = g_strdup_printf (FMTSTR, ctrl->target.targeting->az);
        gtk_label_set_text (GTK_LABEL (ctrl->AzSat), buff);
        g_free (buff);
        buff = g_strdup_printf (FMTSTR, ctrl->target.targeting->el);
        gtk_label_set_text (GTK_LABEL (ctrl->ElSat), buff);
        g_free (buff);
        
        update_count_down (ctrl, t);

        /*if the current pass is too far away*/
        if ((ctrl->target.pass!=NULL)&& (ctrl->qth!=NULL))
            if (qth_small_dist(ctrl->qth,ctrl->target.pass->qth_comp)>1.0){
                free_pass (ctrl->target.pass);
                ctrl->target.pass=NULL;
                ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, t, 3.0);
                if (ctrl->target.pass) {
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
                }
            }
        
        /* update next pass if necessary */
        if (ctrl->target.pass != NULL) {
            /*if we are not in the current pass*/
            if ((ctrl->target.pass->aos>t)||(ctrl->target.pass->los<t)){
                /* the pass may not have met the minimum 
                   elevation, calculate the pass and plot it*/
                if (ctrl->target.targeting->el >= 0.0) {
                    /*inside an unexpected/unpredicted pass*/
                    free_pass (ctrl->target.pass);
                    ctrl->target.pass=NULL;
                    ctrl->target.pass = get_current_pass (ctrl->target.targeting, ctrl->qth, t);
                    set_flipped_pass(ctrl);
                    gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
                } else if ((ctrl->target.targeting->aos-ctrl->target.pass->aos)>(ctrl->delay/secday/1000/4.0)) {
                    /*the target is expected to appear in a new pass 
                      sufficiently later after the current pass says*/
                    
                    /*converted milliseconds to gpredict time and took a 
                      fraction of it as a threshold for deciding a new pass*/
                    
                    /*if the next pass is not the one for the target*/
                    free_pass (ctrl->target.pass);
                    ctrl->target.pass=NULL;
                    ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
                }
            } else {
                /* inside a pass and target dropped below the 
                   horizon so look for a new pass */
                if (ctrl->target.targeting->el < 0.0) {
                    free_pass (ctrl->target.pass);
                    ctrl->target.pass=NULL;
                    ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
                }
            }
        }
        else {
            /* we don't have any current pass; store the current one */
            if (ctrl->target.targeting->el > 0.0) {
                ctrl->target.pass = get_current_pass (ctrl->target.targeting, ctrl->qth, t);
            }
            else {
                ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, t, 3.0);
            }
            set_flipped_pass(ctrl);
            /* update polar plot */
            gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
        }
    }
}


/** \brief Create azimuth control widgets.
 * \param ctrl Pointer to the GtkRotCtrl widget.
 * 
 * This function creates and initialises the widgets for controlling the
 * azimuth of the the rotator.
 */
static
        GtkWidget *create_az_widgets (GtkRotCtrl *ctrl)
{
    GtkWidget   *frame;
    GtkWidget   *table;
    GtkWidget   *label;
    
    
    frame = gtk_frame_new (_("Azimuth"));
    
    table = gtk_table_new (2, 2, FALSE);
    gtk_container_set_border_width (GTK_CONTAINER (table), 5);
    gtk_table_set_col_spacings (GTK_TABLE (table), 5);
    gtk_table_set_row_spacings (GTK_TABLE (table), 5);
    gtk_container_add (GTK_CONTAINER (frame), table);
    
    ctrl->AzSet = gtk_rot_knob_new (0.0, 360.0, 180.0);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->AzSet, 0, 2, 0, 1);

    label = gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (label), _("Read:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach (GTK_TABLE (table), label, 0, 1, 1, 2,
                      GTK_SHRINK, GTK_SHRINK, 10, 0);
    
    ctrl->AzRead = gtk_label_new (" --- ");
    gtk_misc_set_alignment (GTK_MISC (ctrl->AzRead), 0.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->AzRead, 1, 2, 1, 2);
    
    return frame;
}


/** \brief Create elevation control widgets.
 * \param ctrl Pointer to the GtkRotCtrl widget.
 * 
 * This function creates and initialises the widgets for controlling the
 * elevation of the the rotator.
 */
static
        GtkWidget *create_el_widgets (GtkRotCtrl *ctrl)
{
    GtkWidget   *frame;
    GtkWidget   *table;
    GtkWidget   *label;

    
    frame = gtk_frame_new (_("Elevation"));

    table = gtk_table_new (2, 2, FALSE);
    gtk_container_set_border_width (GTK_CONTAINER (table), 5);
    gtk_table_set_col_spacings (GTK_TABLE (table), 5);
    gtk_table_set_row_spacings (GTK_TABLE (table), 5);
    gtk_container_add (GTK_CONTAINER (frame), table);
    
    ctrl->ElSet = gtk_rot_knob_new (0.0, 90.0, 45.0);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->ElSet, 0, 2, 0, 1);

    label = gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (label), _("Read: "));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach (GTK_TABLE (table), label, 0, 1, 1, 2,
                      GTK_SHRINK, GTK_SHRINK, 10, 0);
    
    ctrl->ElRead = gtk_label_new (" --- ");
    gtk_misc_set_alignment (GTK_MISC (ctrl->ElRead), 0.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->ElRead, 1, 2, 1, 2);

    return frame;
}

/** \brief Create target widgets.
 * \param ctrl Pointer to the GtkRotCtrl widget.
 */
static
        GtkWidget *create_target_widgets (GtkRotCtrl *ctrl)
{
    GtkWidget *frame,*table,*satsel;
    GtkTreeViewColumn *sat_nickname, *checkbox, *minCommunication;
    GtkCellRenderer *rend_sat_nickname, *rend_checkbox, *rend_minCommunication;
    GtkWidget *tree;
    GtkTreeIter iter;
    guint i, num_sats;
    sat_t *sat = NULL;
    
    table = gtk_table_new (6, 5, FALSE);
    gtk_container_set_border_width (GTK_CONTAINER (table), 5);
    gtk_table_set_col_spacings (GTK_TABLE (table), 5);
    gtk_table_set_row_spacings (GTK_TABLE (table), 5);

    /* sat selector */
    ctrl->checkSatsList =  gtk_list_store_new(N_COLUMN, G_TYPE_STRING, G_TYPE_BOOLEAN, G_TYPE_STRING);
    
    // set elements to store in the list
    num_sats = g_slist_length (ctrl->sats);
    for (i = 0; i < num_sats; i++) {
        sat = SAT (g_slist_nth_data (ctrl->sats, i));
        if (sat) {
            gtk_list_store_append(ctrl->checkSatsList, &iter);
            gtk_list_store_set(ctrl->checkSatsList, &iter, TEXT_COLUMN, sat->nickname, TOGGLE_COLUMN, FALSE, QNT_COLUMN, "0.000000", -1);
            ctrl->target.sats[i] = NOT_IN_PRIORITY_QUEUE;
            ctrl->target.minCommunication[i] = 0.0f;
        }
    }
    tree = gtk_tree_view_new_with_model(GTK_TREE_MODEL(ctrl->checkSatsList));
    
    // Create render
    rend_sat_nickname = gtk_cell_renderer_text_new();
    rend_checkbox = gtk_cell_renderer_toggle_new();
    rend_minCommunication = gtk_cell_renderer_text_new();
    
    // Set the column view
    sat_nickname = gtk_tree_view_column_new_with_attributes("Target objects", rend_sat_nickname, "text", TEXT_COLUMN, NULL);
    checkbox = gtk_tree_view_column_new_with_attributes("Selected objects", rend_checkbox, "active", TOGGLE_COLUMN, NULL);
    minCommunication = gtk_tree_view_column_new_with_attributes("Min Communication Time (s)", rend_minCommunication, "text",  QNT_COLUMN, NULL);
    
    // Set properties to the objects
    g_object_set(rend_minCommunication, "editable", TRUE, NULL);
    g_object_set(rend_minCommunication, "xalign", 0.5, NULL); 

    // Appends the columns
    gtk_tree_view_append_column(GTK_TREE_VIEW(tree), sat_nickname);
    gtk_tree_view_append_column(GTK_TREE_VIEW(tree), checkbox);
    gtk_tree_view_append_column(GTK_TREE_VIEW(tree), minCommunication);
 //   gtk_tree_view_column_set_clickable(checkbox,"clickable");

    // Sets the viewmode
    gtk_tree_view_set_model(GTK_TREE_VIEW(tree), GTK_TREE_MODEL(ctrl->checkSatsList));

    // Creates a scrolling window 
    satsel = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(satsel), GTK_POLICY_NEVER, GTK_POLICY_NEVER);
    gtk_container_add(GTK_CONTAINER(satsel), tree);
    gtk_table_attach_defaults(GTK_TABLE (table), satsel, 0, 3, 0, 6);
    
    // Callback function
    g_signal_connect (rend_checkbox, "toggled", G_CALLBACK (sat_selected_cb), ctrl);
    g_signal_connect (rend_minCommunication, "edited", G_CALLBACK(sat_changed_minCommunication_cb), ctrl);

    /* satellite priority */
    ctrl->prioritySatsList =  gtk_list_store_new(N_COLUMN_PRIORITY, G_TYPE_STRING);
    tree = gtk_tree_view_new_with_model(GTK_TREE_MODEL(ctrl->prioritySatsList));
    
    // Create render
    rend_sat_nickname = gtk_cell_renderer_text_new();
    
    // Set the column view
    sat_nickname = gtk_tree_view_column_new_with_attributes("Change Priority of Satellite", rend_sat_nickname, "text", TEXT_COLUMN, NULL);
    
    // Appends the columns
    gtk_tree_view_append_column(GTK_TREE_VIEW(tree), sat_nickname);
    
    // Sets the viewmode
    gtk_tree_view_set_model(GTK_TREE_VIEW(tree), GTK_TREE_MODEL(ctrl->prioritySatsList));
    
    // Creates a scrolling window 
    satsel = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(satsel), GTK_POLICY_NEVER, GTK_POLICY_NEVER); 
    gtk_container_add(GTK_CONTAINER(satsel), tree);
    gtk_table_attach_defaults(GTK_TABLE (table), satsel, 3, 4, 0, 6);
    
    frame = gtk_frame_new (_("Target"));
    gtk_container_add (GTK_CONTAINER (frame), table);
    
    
    return frame;
}


static GtkWidget *
        create_conf_widgets (GtkRotCtrl *ctrl)
{
    GtkWidget *frame,*table,*label,*timer,*toler, *track;
    GDir        *dir = NULL;   /* directory handle */
    GError      *error = NULL; /* error flag and info */
    gchar       *dirname;      /* directory name */
    gchar      **vbuff;
    const gchar *filename;     /* file name */
    gchar       *rotname;
    gchar *buff;
    
    buff = g_strdup_printf (FMTSTR, 0.0);
    
    table = gtk_table_new (6, 4, FALSE);
    gtk_container_set_border_width (GTK_CONTAINER (table), 5);
    gtk_table_set_col_spacings (GTK_TABLE (table), 5);
    gtk_table_set_row_spacings (GTK_TABLE (table), 5);
    
    
    label = gtk_label_new (_("Device:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 0, 1, 0, 1);
    
    ctrl->DevSel = gtk_combo_box_new_text ();
    gtk_widget_set_tooltip_text (ctrl->DevSel, _("Select antenna rotator device"));
    
    /* open configuration directory */
    dirname = get_hwconf_dir ();
    
    dir = g_dir_open (dirname, 0, &error);
    if (dir) {
        /* read each .rot file */
        GSList *rots=NULL;
        gint i;
        gint n;
        while ((filename = g_dir_read_name (dir))) {
            
            if (g_str_has_suffix (filename, ".rot")) {
                
                vbuff = g_strsplit (filename, ".rot", 0);
                rots=g_slist_insert_sorted(rots,g_strdup(vbuff[0]),(GCompareFunc)rot_name_compare);
                g_strfreev (vbuff);
            }
        }
        n = g_slist_length (rots);
        for (i = 0; i < n; i++) {
            rotname = g_slist_nth_data (rots, i);
            if (rotname) {
                gtk_combo_box_append_text (GTK_COMBO_BOX (ctrl->DevSel), rotname);
                g_free(rotname);
            }
        }
        g_slist_free(rots);
    }
    else {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Failed to open hwconf dir (%s)"),
                     __FILE__, __LINE__, error->message);
        g_clear_error (&error);
    }

    g_free (dirname);
    g_dir_close (dir);

    gtk_combo_box_set_active (GTK_COMBO_BOX (ctrl->DevSel), 0);
    g_signal_connect (ctrl->DevSel, "changed", G_CALLBACK (rot_selected_cb), ctrl);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->DevSel, 1, 3, 0, 1);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label (_("Engage"));
    gtk_widget_set_tooltip_text (ctrl->LockBut, _("Engage the selected rotor device"));
    g_signal_connect (ctrl->LockBut, "toggled", G_CALLBACK (rot_locked_cb), ctrl);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->LockBut, 3, 4, 0, 1);
  
    /* tracking button */
    track = gtk_toggle_button_new_with_label (_("Track"));
    gtk_widget_set_tooltip_text (track, _("Track the satellite when it is within range"));
    gtk_table_attach_defaults (GTK_TABLE (table), track, 4, 5, 0, 1);
    g_signal_connect (track, "toggled", G_CALLBACK (track_toggle_cb), ctrl);   
  
    /* Current objcet beeing tracked */
    label = gtk_label_new (_("Target:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 0, 1, 1, 2);

    ctrl->track_sat = gtk_label_new (_(" --- "));
    gtk_misc_set_alignment (GTK_MISC (ctrl->track_sat), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->track_sat, 1, 2, 1, 2);

    /* Azimuth */
    label = gtk_label_new (_("Az:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 0, 1, 2, 3);
    
    ctrl->AzSat = gtk_label_new (buff);
    gtk_misc_set_alignment (GTK_MISC (ctrl->AzSat), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->AzSat, 2, 3, 2, 3);
    
    /* Elevation */
    label = gtk_label_new (_("El:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 0, 1, 3, 4);
    
    ctrl->ElSat = gtk_label_new (buff);
    gtk_misc_set_alignment (GTK_MISC (ctrl->ElSat), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->ElSat, 2, 3, 3, 4);
 
    /* count down */
    label = gtk_label_new (_("\316\224T:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 0, 1, 4, 5);
    ctrl->SatCnt = gtk_label_new ("00:00:00");
    gtk_misc_set_alignment (GTK_MISC (ctrl->SatCnt), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), ctrl->SatCnt, 2, 3, 4, 5);
    
    /* Timeout */
    label = gtk_label_new (_("Cycle:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 3, 4, 2, 3);
    
    timer = gtk_spin_button_new_with_range (1000, 10000, 10);
    gtk_spin_button_set_digits (GTK_SPIN_BUTTON (timer), 0);
    gtk_widget_set_tooltip_text (timer,
                                 _("This parameter controls the delay between "\
                                   "commands sent to the rotator."));
    gtk_spin_button_set_value (GTK_SPIN_BUTTON (timer), ctrl->delay);
    g_signal_connect (timer, "value-changed", G_CALLBACK (delay_changed_cb), ctrl);
    gtk_table_attach (GTK_TABLE (table), timer, 4, 5, 2, 3,
                      GTK_FILL, GTK_FILL, 0, 0);
    
    label = gtk_label_new (_("msec"));
    gtk_misc_set_alignment (GTK_MISC (label), 0.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 4, 5, 2, 3);

    /* Tolerance */
    label = gtk_label_new (_("Tolerance:"));
    gtk_misc_set_alignment (GTK_MISC (label), 1.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 3, 4, 3, 4);
    
    toler = gtk_spin_button_new_with_range (0.01, 50.0, 0.01);
    gtk_spin_button_set_digits (GTK_SPIN_BUTTON (toler), 2);
    gtk_widget_set_tooltip_text (toler,
                                 _("This parameter controls the tolerance between "\
                                   "the target and rotator values for the rotator.\n"\
                                   "If the difference between the target and rotator values "\
                                   "is smaller than the tolerance, no new commands are sent"));
    gtk_spin_button_set_value (GTK_SPIN_BUTTON (toler), ctrl->tolerance);
    g_signal_connect (toler, "value-changed", G_CALLBACK (toler_changed_cb), ctrl);
    gtk_table_attach (GTK_TABLE (table), toler, 4, 5, 3, 4,
                      GTK_FILL, GTK_FILL, 0, 0);
    
    
    label = gtk_label_new (_("deg"));
    gtk_misc_set_alignment (GTK_MISC (label), 0.0, 0.5);
    gtk_table_attach_defaults (GTK_TABLE (table), label, 4, 5, 3, 4);
    

    /* load initial rotator configuration */
    rot_selected_cb (GTK_COMBO_BOX (ctrl->DevSel), ctrl);
    
    frame = gtk_frame_new (_("Settings"));
    gtk_container_add (GTK_CONTAINER (frame), table);
    
    g_free (buff);

    return frame;
}


/** \brief Create target widgets.
 * \param ctrl Pointer to the GtkRotCtrl widget.
 */
static
        GtkWidget *create_plot_widget (GtkRotCtrl *ctrl)
{
    GtkWidget *frame;
    
    ctrl->plot = gtk_polar_plot_new (ctrl->qth, ctrl->target.pass);
    
    frame = gtk_frame_new (NULL);
    gtk_container_add (GTK_CONTAINER (frame), ctrl->plot);
    
    return frame;
}


/** \brief Copy satellite from hash table to singly linked list.
 */
static void
        store_sats (gpointer key, gpointer value, gpointer user_data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL( user_data);
    sat_t        *sat = SAT (value);
    
    (void) key; /* avoid unused variable warning */
    
    ctrl->sats = g_slist_insert_sorted (ctrl->sats, sat, (GCompareFunc)sat_name_compare);
}



/** \brief Manage satellite selections
 * \param cell_render Pointer to the GtkCellRenderToggle.
 * \param path_str String of the i-th changed cell
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user selects a new satellite.
 */
static void
        sat_selected_cb (GtkCellRendererToggle *cell_renderer, gchar *path_str, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    GtkTreePath *path = gtk_tree_path_new_from_string (path_str);
    sat_t * sat;
    char *path_str_priority;
    unsigned int ndigits, index_priority_list;
    GtkTreeIter iter;
    gboolean enable;
    gint i;

    gtk_tree_model_get_iter (GTK_TREE_MODEL(ctrl->checkSatsList), &iter, path);
    gtk_tree_model_get (GTK_TREE_MODEL(ctrl->checkSatsList), &iter, TOGGLE_COLUMN, &enable, -1);
    enable ^= 1;

    gtk_list_store_set(ctrl->checkSatsList, &iter, TOGGLE_COLUMN, enable, -1);

    sscanf(path_str, " %d ",&i);
    if(enable && i >= 0){
        /* Show new added satellite in the priority queue */
        sat = SAT (g_slist_nth_data (ctrl->sats, i));
        if (sat) {
            gtk_list_store_append(ctrl->prioritySatsList , &iter);
            gtk_list_store_set(ctrl->prioritySatsList, &iter, TEXT_COLUMN, sat->nickname, -1);
            /* Adds the satellite to the priorityQueue */
            append_elem_priority_queue(ctrl, i);
        }
    }
    else if(!enable && i >= 0){
        index_priority_list = get_elem_index_priority_queue(ctrl, i);
        remove_elem_priority_queue(ctrl, i);
        // Gets the new iter
        ndigits = log10(index_priority_list+1)+1;
        path_str_priority = malloc(sizeof(char)*ndigits);
        sprintf(path_str_priority, "%d", index_priority_list);
        gtk_tree_path_free (path);
        path = gtk_tree_path_new_from_string (path_str_priority);
        gtk_tree_model_get_iter (GTK_TREE_MODEL(ctrl->prioritySatsList), &iter, path);
        // Removes the row
        gtk_list_store_remove(ctrl->prioritySatsList, &iter);
        free(path_str_priority);
    }

    /* Updates the next target*/
    update_tracked_elem(ctrl);

    if(ctrl->target.targeting != NULL){
        /* update next pass */
     /*   if (ctrl->target.targeting != NULL)
            free_pass (ctrl->target.pass);

        if (ctrl->target.targeting->el > 0.0)
            ctrl->target.pass = get_current_pass (ctrl->target.targeting, ctrl->qth, ctrl->t);
        else
            ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, ctrl->t, 3.0);

        set_flipped_pass(ctrl);*/

        if(ctrl->plot != NULL){
            /* Plots a new pass only if there is a new element added */
            gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
        }
    }

    /* Frees the path */
    gtk_tree_path_free (path);
}

/** \brief Set new minimal time of communication to selected sattelite
 * \param cellrederertext Pointer to the GtkCellRendererText. 
 * \param arg1 String of the i-th changed cell
 * \param arg2 String of the new text entered by the user
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user changes the minimal time of communication of a satellite.
 */
static void 
    sat_changed_minCommunication_cb(GtkCellRendererText *cellrenderertext, gchar *arg1, gchar *arg2, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    GtkTreePath *path = gtk_tree_path_new_from_string (arg1);
    char *new_num_str;
    GtkTreeIter iter;
    float num = 0.0f; 
    int res = 0, i, strlength = strlen(arg2);
    
    /* Transforms a number of the type *,* in a number of the type *.* */
    for(i=0; i < strlength; i++)
        if(arg2[i] == ',')
            arg2[i] = '.';
    /* Reads the number */
    res = sscanf(arg2, "%f", &num);
    new_num_str = malloc(sizeof(char) * strlength);
    if(num >= 0.0f && res == 1){
        sprintf(new_num_str, "%f", num);
    }
    else{
        num = 0.0f;
        sprintf(new_num_str, "%f", num);
    }
    /* Sets the new number to the tree model */
    gtk_tree_model_get_iter (GTK_TREE_MODEL(ctrl->checkSatsList), &iter, path);
    gtk_list_store_set(ctrl->checkSatsList, &iter, QNT_COLUMN, new_num_str, -1);

    /* Sets the minCommunication to the i-th satellite */
    sscanf(arg1, "%d", &i);
    ctrl->target.minCommunication[i] = num;

    /* Resets the next passes if there is a satellite*/
    update_tracked_elem(ctrl);
}

/** \brief Manage toggle signals (tracking)
 * \param button Pointer to the GtkToggle button.
 * \param data Pointer to the GtkRotCtrl widget.
 */
static void
        track_toggle_cb (GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    
    ctrl->tracking = gtk_toggle_button_get_active (button);
}


/** \brief Manage cycle delay changes.
 * \param spin Pointer to the spin button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user changes the value of the
 * cycle delay.
 */
static void
        delay_changed_cb (GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    
    
    ctrl->delay = (guint) gtk_spin_button_get_value (spin);

    if (ctrl->timerid > 0) 
        g_source_remove (ctrl->timerid);

    ctrl->timerid = g_timeout_add (ctrl->delay, rot_ctrl_timeout_cb, ctrl);
}



/** \brief Manage tolerance changes.
 * \param spin Pointer to the spin button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user changes the value of the
 * tolerance.
 */
static void
        toler_changed_cb (GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    
    ctrl->tolerance = gtk_spin_button_get_value (spin);
}


/** \brief New rotor device selected.
 * \param box Pointer to the rotor selector combo box.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user selects a new rotor controller
 * device.
 */
static void
        rot_selected_cb (GtkComboBox *box, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    
    /* free previous configuration */
    if (ctrl->conf != NULL) {
        g_free (ctrl->conf->name);
        g_free (ctrl->conf->host);
        g_free (ctrl->conf);
    }
    
    ctrl->conf = g_try_new (rotor_conf_t, 1);
    if (ctrl->conf == NULL) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Failed to allocate memory for rotator config"),
                     __FILE__, __LINE__);
        return;
    }
    
    /* load new configuration */
    ctrl->conf->name = gtk_combo_box_get_active_text (box);
    if (rotor_conf_read (ctrl->conf)) {
        sat_log_log (SAT_LOG_LEVEL_INFO,
                     _("Loaded new rotator configuration %s"),
                     ctrl->conf->name);
        
        /* update new ranges of the Az and El controller widgets */
        gtk_rot_knob_set_range (GTK_ROT_KNOB (ctrl->AzSet), ctrl->conf->minaz, ctrl->conf->maxaz);
        gtk_rot_knob_set_range (GTK_ROT_KNOB (ctrl->ElSet), ctrl->conf->minel, ctrl->conf->maxel);

        /*Update flipped when changing rotor if there is a plot*/
        set_flipped_pass(ctrl);
    }
    else {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Failed to load rotator configuration %s"),
                     __FILE__, __LINE__, ctrl->conf->name);

        g_free (ctrl->conf->name);
        if (ctrl->conf->host)
            g_free (ctrl->conf->host);
        g_free (ctrl->conf);
        ctrl->conf = NULL;
    }
}



/** \brief Rotor locked.
 * \param button Pointer to the "Engage" button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user toggles the "Engage" button.
 */
static void
        rot_locked_cb (GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    
    if (!gtk_toggle_button_get_active (button)) {
        gtk_widget_set_sensitive (ctrl->DevSel, TRUE);
        ctrl->engaged = FALSE;
        close_rotctld_socket(&(ctrl->sock));
        gtk_label_set_text (GTK_LABEL (ctrl->AzRead), "---");
        gtk_label_set_text (GTK_LABEL (ctrl->ElRead), "---");
    }
    else {
        if (ctrl->conf == NULL) {
            /* we don't have a working configuration */
            sat_log_log (SAT_LOG_LEVEL_ERROR,
                         _("%s: Controller does not have a valid configuration"),
                         __FUNCTION__);
            return;
        }
        gtk_widget_set_sensitive (ctrl->DevSel, FALSE);
        ctrl->engaged = TRUE;
        open_rotctld_socket(ctrl);
        ctrl->wrops = 0;
        ctrl->rdops = 0;
    }
}

/** \brief Rotator controller timeout function
 * \param data Pointer to the GtkRotCtrl widget.
 * \return Always TRUE to let the timer continue.
 */
static gboolean
        rot_ctrl_timeout_cb (gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL (data);
    gdouble rotaz=0.0, rotel=0.0;
    gdouble setaz=0.0, setel=45.0;
    gdouble cmdaz=0.0, cmdel=45.0;
    gchar *text;
    gboolean error = FALSE;
    sat_t sat_working, *sat;
    /*parameters for path predictions*/
    gdouble time_delta;
    gdouble step_size;
    
    
    if (g_static_mutex_trylock(&(ctrl->busy))==FALSE) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,_("%s missed the deadline"),__FUNCTION__);
        return TRUE;
    }
    
    /* Update the tracking object */
    update_tracked_elem(ctrl);

    /* If we are tracking and the target satellite is within
       range, set the rotor position controller knob values to
       the target values. If the target satellite is out of range
       set the rotor controller to 0 deg El and to the Az where the
       target sat is expected to come up or where it last went down
    */
    if (ctrl->tracking && ctrl->target.targeting) {
        if (ctrl->target.targeting->el < 0.0) {
            if (ctrl->target.pass != NULL) {
                if (ctrl->t < ctrl->target.pass->aos) {
                    setaz=ctrl->target.pass->aos_az;
                    setel=0;
                } else if (ctrl->t > ctrl->target.pass->los) {
                    setaz=ctrl->target.pass->los_az;
                    setel=0;
                }
            }
        }
        else { 
            setaz=ctrl->target.targeting->az;
            setel=ctrl->target.targeting->el;
        }
        /* if this is a flipped pass and the rotor supports it*/
        if ((ctrl->flipped)&&(ctrl->conf->maxel>=180.0)){
            setel=180-setel;
            if (setaz>180)
                setaz-=180;
            else
                setaz+=180;
        }
        if ((ctrl->conf->aztype == ROT_AZ_TYPE_180) && (setaz > 180.0)) {
            setaz = setaz- 360.0;
        }
        if (!(ctrl->engaged)) {
            gtk_rot_knob_set_value (GTK_ROT_KNOB (ctrl->AzSet), setaz);
            gtk_rot_knob_set_value (GTK_ROT_KNOB (ctrl->ElSet), setel);
        }

    } else {
        setaz = gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->AzSet));
        setel = gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->ElSet));
    }


    if ((ctrl->engaged) && (ctrl->conf != NULL)) {
        
        /* read back current value from device */
        if (get_pos (ctrl, &rotaz, &rotel)) {

            /* update display widgets */
            text = g_strdup_printf ("%.2f\302\260", rotaz);
            gtk_label_set_text (GTK_LABEL (ctrl->AzRead), text);
            g_free (text);
            text = g_strdup_printf ("%.2f\302\260", rotel);
            gtk_label_set_text (GTK_LABEL (ctrl->ElRead), text);
            g_free (text);
            
            if ((ctrl->conf->aztype == ROT_AZ_TYPE_180) && (rotaz < 0.0)) {
                gtk_polar_plot_set_rotor_pos (GTK_POLAR_PLOT (ctrl->plot), rotaz+360.0, rotel);
            }
            else {
                gtk_polar_plot_set_rotor_pos (GTK_POLAR_PLOT (ctrl->plot), rotaz, rotel);
            }
        }
        else {
            gtk_label_set_text (GTK_LABEL (ctrl->AzRead), _("ERROR"));
            gtk_label_set_text (GTK_LABEL (ctrl->ElRead), _("ERROR"));
            error = TRUE;

            gtk_polar_plot_set_rotor_pos (GTK_POLAR_PLOT (ctrl->plot), -10.0, -10.0);
        }
        
        /* if tolerance exceeded */
        if ((fabs(setaz-rotaz) > ctrl->tolerance) ||
            (fabs(setel-rotel) > ctrl->tolerance)) {
            
            if (ctrl->tracking){
                /*if we are in a pass try to lead the satellite 
                  some so we are not always chasing it*/
                if (ctrl->target.targeting->el>0.0) {
                    /*starting the rotator moving while we do some computation can lead to errors later*/
                    /* 
                       compute a time in the future when the position is 
                       within tolerance so and send the rotor there.
                    */         
                    
                    /*use a working copy so data does not get corrupted*/
                    sat=memcpy(&(sat_working),ctrl->target.targeting,sizeof(sat_t));
                    
                    /*
                      compute az/el in the future that is past end of pass 
                      or exceeds tolerance
                    */
                    if (ctrl->target.pass) {
                        /* the next point is before the end of the pass 
                           if there is one.*/
                        time_delta=ctrl->target.pass->los-ctrl->t;
                    } else {
                        /* otherwise look 20 minutes into the future*/
                        time_delta=1.0/72.0;
                    }

                    /* have a minimum time delta*/
                    step_size = time_delta / 2.0;
                    if (step_size<ctrl->delay/1000.0/(secday)){
                        step_size=ctrl->delay/1000.0/(secday);
                    }
                    /*
                      find a time when satellite is above horizon and at the 
                      edge of tolerance. the final step size needs to be smaller
                      than delay. otherwise the az/el could be further away than
                      tolerance the next time we enter the loop and we end up 
                      pushing ourselves away from the satellite.
                    */
                    while (step_size > (ctrl->delay/1000.0/4.0/(secday))) {
                        predict_calc (sat,ctrl->qth,ctrl->t+time_delta);
                        /*update sat->az and sat->el to account for flips and az range*/
                        if ((ctrl->flipped) && (ctrl->conf->maxel >= 180.0)){
                            sat->el = 180.0-sat->el;
                            if (sat->az > 180.0)
                                sat->az -= 180.0;
                            else
                                sat->az += 180.0;
                        }
                        if ((ctrl->conf->aztype == ROT_AZ_TYPE_180) && (sat->az > 180.0)) { 
                            sat->az = sat->az - 360.0;
                        } 
                        if ((sat->el < 0.0)||(sat->el > 180.0)||
                            (fabs(setaz - sat->az) > (ctrl->tolerance)) ||
                            (fabs(setel - sat->el) > (ctrl->tolerance))) {
                            time_delta -= step_size;
                        } else {
                            time_delta += step_size;
                        }
                        step_size /= 2.0;
                    }
                    setel = sat->el;
                    if (setel < 0.0) {
                        setel = 0.0;
                    }
                    if (setel > 180.0) {
                        setel = 180.0;
                    }
                    setaz = sat->az;
                }
            }
        
			/* If azimuth position is > 180 from current position, send incremental command */

			if(fabs(setaz - rotaz) >= 180.0){
				if(setaz > rotaz){
					cmdaz = rotaz + 90;
				}else{
					cmdaz = rotaz - 90;
				}
			}else{
				cmdaz = setaz;
			}
			cmdel = setel;

            /* send controller values to rotator device */
            /* this is the newly computed value which should be ahead of the current position */
            if (!set_pos (ctrl, cmdaz, cmdel)) {
                error = TRUE;
            } else {
                gtk_rot_knob_set_value (GTK_ROT_KNOB (ctrl->AzSet), setaz);
                gtk_rot_knob_set_value (GTK_ROT_KNOB (ctrl->ElSet), setel);
            }
        }
        
        /* check error status */
        if (!error) {
            /* reset error counter */
            ctrl->errcnt = 0;
        }
        else {
            if (ctrl->errcnt >= MAX_ERROR_COUNT) {
                /* disengage device */
                gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (ctrl->LockBut), FALSE);
                ctrl->engaged = FALSE;
                sat_log_log (SAT_LOG_LEVEL_ERROR,
                             _("%s: MAX_ERROR_COUNT (%d) reached. Disengaging device!"),
                             __FUNCTION__, MAX_ERROR_COUNT);
                ctrl->errcnt = 0;
                //g_print ("ERROR. WROPS: %d   RDOPS: %d\n", ctrl->wrops, ctrl->rdops);
            }
            else {
                /* increment error counter */
                ctrl->errcnt++;
            }
        }
    }
    else {
        /* ensure rotor pos is not visible on plot */
        gtk_polar_plot_set_rotor_pos (GTK_POLAR_PLOT (ctrl->plot), -10.0, -10.0);
    }
    
    
    /* update target object on polar plot */
    if (ctrl->target.targeting != NULL) {
        gtk_polar_plot_set_target_pos (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.targeting->az, ctrl->target.targeting->el);
    }
    
    /* update controller circle on polar plot */
    if (ctrl->conf !=NULL){
        if ((ctrl->conf->aztype == ROT_AZ_TYPE_180) && (setaz < 0.0)) {
            gtk_polar_plot_set_ctrl_pos (GTK_POLAR_PLOT (ctrl->plot),
                                         gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->AzSet))+360.0,
                                         gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->ElSet)));
        }
        else {
            gtk_polar_plot_set_ctrl_pos (GTK_POLAR_PLOT (ctrl->plot),
                                         gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->AzSet)),
                                         gtk_rot_knob_get_value (GTK_ROT_KNOB (ctrl->ElSet)));
        }
    }
    g_static_mutex_unlock(&(ctrl->busy));

    return TRUE;
}


/** \brief Read rotator position from device.
 * \param ctrl Pointer to the GtkRotCtrl widget.
 * \param az The current Az as read from the device
 * \param el The current El as read from the device
 * \return TRUE if the position was successfully retrieved, FALSE if an
 *         error occurred.
 */
static gboolean get_pos (GtkRotCtrl *ctrl, gdouble *az, gdouble *el)
{
    gchar  *buff,**vbuff;
    gchar  buffback[128];
    gboolean retcode;
    
    if ((az == NULL) || (el == NULL)) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: NULL storage."),
                     __FILE__, __LINE__);
        return FALSE;
    }
    
    /* send command */
    buff = g_strdup_printf ("p\x0a");

    retcode=send_rotctld_command(ctrl,buff,buffback,128);
    
    /* try to read answer */
    if (retcode) {
        if (strncmp(buffback,"RPRT",4)==0){
            //retcode=FALSE;
            g_strstrip (buffback);
            sat_log_log (SAT_LOG_LEVEL_ERROR,
                         _("%s:%d: rotctld returned error (%s)"),
                         __FILE__, __LINE__,buffback);

        } else {
            vbuff = g_strsplit (buffback, "\n", 3);
            if ((vbuff[0] !=NULL) && (vbuff[1]!=NULL)){
                *az = g_strtod (vbuff[0], NULL);
                *el = g_strtod (vbuff[1], NULL);
            } else {
                g_strstrip (buffback);
                sat_log_log (SAT_LOG_LEVEL_ERROR,
                             _("%s:%d: rotctld returned bad response (%s)"),
                             __FILE__, __LINE__,buffback);
                //retcode=FALSE;
            }
            
            g_strfreev (vbuff);
        }
    }

    g_free (buff);

    return retcode;
}


/** \brief Send new position to rotator device
 * \param ctrl Pointer to the GtkRotCtrl widget
 * \param az The new Azimuth
 * \param el The new Elevation
 * \return TRUE if the new position has been sent successfully
 *         FALSE if an error occurred
 * 
 * \note The function does not perform any range check since the GtkRotKnob
 * should always keep its value within range.
 */
static gboolean set_pos (GtkRotCtrl *ctrl, gdouble az, gdouble el)
{
    gchar  *buff;
    gchar  buffback[128];
    gchar  azstr[8],elstr[8];
    gboolean retcode;
    gint   retval;
    
    /* send command */
    g_ascii_formatd (azstr, 8, "%7.2f", az);
    g_ascii_formatd (elstr, 8, "%7.2f", el);
    buff = g_strdup_printf ("P %s %s\x0a", azstr, elstr);
    
    retcode=send_rotctld_command(ctrl,buff,buffback,128);
    
    g_free (buff);
    
    if (retcode==TRUE){
        retval=(gint)g_strtod(buffback+4,NULL);
        /*treat errors as soft errors unless there is good reason*/
        /*good reasons come from operator experience or documentation*/
        switch(retval) {
            
        case 0:
            /*no error case*/
            break;
        default:
            /*any other case*/
            /*not sure what is a hard error or soft error*/
            /*over time a database of this is needed*/
            g_strstrip (buffback);
            sat_log_log (SAT_LOG_LEVEL_ERROR,
                         _("%s:%d: rotctld returned error %d with az %f el %f(%s)"),
                         __FILE__, __LINE__, retval, az, el, buffback);
            
            //retcode=FALSE;
            break;
        }
    }

    return (retcode);
}




/** \brief Update count down label.
 * \param[in] ctrl Pointer to the RotCtrl widget.
 * \param[in] t The current time.
 * 
 * This function calculates the new time to AOS/LOS of the currently
 * selected target and updates the ctrl->SatCnt label widget.
 */
static void update_count_down (GtkRotCtrl *ctrl, gdouble t)
{
    gdouble  targettime;
    gdouble  delta;
    gchar   *buff;
    guint    h,m,s;

    
    /* select AOS or LOS time depending on target elevation */
    if (ctrl->target.targeting->el < 0.0)
        targettime = ctrl->target.targeting->aos;
    else
        targettime = ctrl->target.targeting->los;
    
    delta = targettime - t;
    
    /* convert julian date to seconds */
    s = (guint) (delta * 86400);

    /* extract hours */
    h = (guint) floor (s/3600);
    s -= 3600*h;

    /* extract minutes */
    m = (guint) floor (s/60);
    s -= 60*m;

    if (h > 0) 
        buff = g_strdup_printf ("%02d:%02d:%02d", h, m, s);
    else
        buff = g_strdup_printf ("%02d:%02d",  m, s);

    gtk_label_set_text (GTK_LABEL (ctrl->SatCnt), buff);

    g_free (buff);

}


/** \brief Check that we have at least one .rot file */
static gboolean have_conf ()
{
    GDir        *dir = NULL;   /* directory handle */
    GError      *error = NULL; /* error flag and info */
    gchar       *dirname;      /* directory name */
    const gchar *filename;     /* file name */
    gint         i = 0;

    
    /* open configuration directory */
    dirname = get_hwconf_dir ();
    
    dir = g_dir_open (dirname, 0, &error);
    if (dir) {
        /* read each .rot file */
        while ((filename = g_dir_read_name (dir))) {
            
            if (g_str_has_suffix (filename, ".rot")) {
                i++;
                /*once we have one we need nothing else*/
                break;
            }
        }
    }
    else {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Failed to open hwconf dir (%s)"),
                     __FILE__, __LINE__, error->message);
        g_clear_error (&error);
    }

    g_free (dirname);
    g_dir_close (dir);
    
    return (i > 0) ? TRUE : FALSE;
}

/** \brief open the rotcld socket. return true if successful false otherwise.*/

static gboolean open_rotctld_socket (GtkRotCtrl * ctrl) {
    struct sockaddr_in ServAddr;
    struct hostent *h;
    gint status;

    ctrl->sock=socket(PF_INET,SOCK_STREAM,IPPROTO_TCP);
    if (ctrl->sock < 0) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s: Failed to create socket"),
                     __FUNCTION__);
        ctrl->sock = 0;
        return FALSE;
    }
    else {
        sat_log_log (SAT_LOG_LEVEL_DEBUG,
                     _("%s: Network socket created successfully"),
                     __FUNCTION__);
    }

    memset(&ServAddr, 0, sizeof(ServAddr));     /* Zero out structure */
    ServAddr.sin_family = AF_INET;             /* Internet address family */
    h = gethostbyname(ctrl->conf->host);
    memcpy((char *) &ServAddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
    ServAddr.sin_port = htons(ctrl->conf->port); /* Server port */

    /* establish connection */
    status = connect(ctrl->sock, (struct sockaddr *) &ServAddr, sizeof(ServAddr));
    if (status < 0) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s: Failed to connect to %s:%d"),
                     __FUNCTION__, ctrl->conf->host, ctrl->conf->port);
        ctrl->sock = 0;
        return FALSE;
    }
    else {
        sat_log_log (SAT_LOG_LEVEL_DEBUG,
                     _("%s: Connection opened to %s:%d"),
                     __FUNCTION__, ctrl->conf->host, ctrl->conf->port);
    }

    return TRUE;
}


/*close a rotcld socket. First send a q command to cleanly shut down rotctld*/
static gboolean close_rotctld_socket (gint *sock) {
  gint written;
  /*shutdown the rigctld connect*/
  written = send(*sock, "q\x0a", 2, 0);
  if (written != 2) {
      sat_log_log (SAT_LOG_LEVEL_ERROR,
                   _("%s:%s: Sent 2 bytes but sent %d."),
                   __FILE__, __FUNCTION__, written);
  }
#ifndef WIN32
  shutdown (*sock, SHUT_RDWR);
  close (*sock);
#else
  shutdown (*sock, SD_BOTH);
  closesocket (*sock);
#endif
  
  *sock=0;
  
  return TRUE;
}

/** \brief  Send a command to rigctld
 *   Inputs are a controller, a string command, and a buffer and length for returning the output from rigctld.
 */

gboolean send_rotctld_command(GtkRotCtrl *ctrl, gchar *buff, gchar *buffout, gint sizeout)
{
    gint    written;
    gint    size;

    size = strlen(buff);
    
    //sat_log_log (SAT_LOG_LEVEL_DEBUG,
    //             _("%s:%s: Sending %d bytes as %s."),
    //             __FILE__, __FUNCTION__, size, buff);


    /* send command */
    written = send(ctrl->sock, buff, size, 0);
    if (written != size) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s: SIZE ERROR %d / %d"),
                     __FUNCTION__, written, size);
    }
    if (written == -1) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s: rotctld Socket Down"),
                     __FUNCTION__);
        return FALSE;
    }

    /* try to read answer */
    size = recv (ctrl->sock, buffout, sizeout, 0);

    if (size == -1) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s: rotctld Socket Down"),
                     __FUNCTION__);
        return FALSE;
    }  

    buffout[size]='\0';
    if (size == 0) {
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%s: Got 0 bytes from rotctld"),
                     __FILE__, __FUNCTION__);
    }
    else {
        //sat_log_log (SAT_LOG_LEVEL_DEBUG,
        //             _("%s:%s: Read %d bytes as %s from rotctld"),
        //             __FILE__, __FUNCTION__, size, buffout);
        
    }

    ctrl->wrops++;
    
    return TRUE;
}

/** \brief  Compare Satellite Names.
 *simple function to sort the list of satellites in the combo box.
 */
static gint sat_name_compare (sat_t* a,sat_t*b){
    return (gpredict_strcmp(a->nickname,b->nickname));
}


/** \brief  Compare Rotator Names.
 */
static gint rot_name_compare (const gchar* a,const gchar *b){
    return (gpredict_strcmp(a,b));
}


/** \brief  Compute if a pass is flipped or not.  this is a function of the rotator and the particular pass. 
 */
static gboolean is_flipped_pass (pass_t * pass,rot_az_type_t type){
    gdouble max_az = 0,min_az = 0;
    gdouble caz,last_az=pass->aos_az;
    guint num,i;
    pass_detail_t      *detail;
    gboolean retval=FALSE;

    num = g_slist_length (pass->details);
    if (type==ROT_AZ_TYPE_360) {
        min_az = 0;
        max_az = 360;
    } 
    else if (type==ROT_AZ_TYPE_180) {
        min_az = -180;
        max_az = 180;
    }
    
    /* Assume that min_az and max_az are atleat 360 degrees apart*/
    /*get the azimuth that is in a settable range*/
    while (last_az>max_az) {
        last_az-=360;
    } 
    while (last_az<min_az) {
        last_az+=360;
    }
    
    if (num>1) {
        for (i = 1; i < num-1; i++) {
            detail = PASS_DETAIL(g_slist_nth_data (pass->details, i));
            caz=detail->az;
            while (caz>max_az) {
                caz-=360;
            } 
            while (caz<min_az) {
                caz+=360;
            }
            if (fabs(caz-last_az)>180) {
                retval=TRUE;
            }
            last_az=caz;
            
        }
    }
    caz=pass->los_az;
    while (caz>max_az) {
        caz-=360;
    } 
    while (caz<min_az) {
        caz+=360;
    }
    if (fabs(caz-last_az)>180) {
        retval=TRUE;
    }
    
    return retval;
}

static inline void set_flipped_pass (GtkRotCtrl* ctrl){
    if (ctrl->conf)
        if (ctrl->target.pass){
            ctrl->flipped=is_flipped_pass(ctrl->target.pass,ctrl->conf->aztype);
        }

}

/*
 * Finish CODE here
 */
static void
        append_elem_priority_queue(GtkRotCtrl* ctrl, int i)
{
    int n;

    ctrl->target.numSatToTrack++;
    n = ctrl->target.numSatToTrack;
    if(n == 1){
        /* update targeting satellite */
        ctrl->target.targeting = SAT (g_slist_nth_data (ctrl->sats, i));
        /* update next pass */
        if (ctrl->target.targeting != NULL)
            free_pass (ctrl->target.pass);

        if (ctrl->target.targeting->el > 0.0)
            ctrl->target.pass = get_current_pass (ctrl->target.targeting, ctrl->qth, ctrl->t);
        else
            ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, ctrl->t, 3.0);

        set_flipped_pass(ctrl);

        if(ctrl->plot != NULL){
            /* Plots a new pass only if there is a new element added */
            gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
        }
    }
    ctrl->target.priorityQueue[n-1] = i;
    ctrl->target.sats[i] = n-1;
}

static void
        remove_elem_priority_queue(GtkRotCtrl* ctrl, int i)
{
    int j = ctrl->target.sats[i];
       
    if(j == NOT_IN_PRIORITY_QUEUE){
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Error in priority list"),
                     __FILE__, __LINE__);
    }
    else{
        ctrl->target.sats[ctrl->target.priorityQueue[j]] = NOT_IN_PRIORITY_QUEUE;
        for(; j < ctrl->target.numSatToTrack-1; j++){
            ctrl->target.priorityQueue[j] = ctrl->target.priorityQueue[j+1];
            ctrl->target.sats[ctrl->target.priorityQueue[j]] = j;
        }
        ctrl->target.priorityQueue[j] = NOT_IN_PRIORITY_QUEUE;

        ctrl->target.numSatToTrack--;

        if(ctrl->target.numSatToTrack == 0){
            ctrl->target.targeting = NULL;
            ctrl->target.pass = NULL;
        }
        else{
            /* update targeting satellite */
            ctrl->target.targeting = SAT (g_slist_nth_data (ctrl->sats, ctrl->target.priorityQueue[0]));
            /* update next pass */
            if (ctrl->target.targeting != NULL)
                free_pass (ctrl->target.pass);

            if (ctrl->target.targeting->el > 0.0)
                ctrl->target.pass = get_current_pass (ctrl->target.targeting, ctrl->qth, ctrl->t);
            else
                ctrl->target.pass = get_pass (ctrl->target.targeting, ctrl->qth, ctrl->t, 3.0);

            set_flipped_pass(ctrl);

            if(ctrl->plot != NULL){
                /* Plots a new pass only if there is a new element added */
                gtk_polar_plot_set_pass (GTK_POLAR_PLOT (ctrl->plot), ctrl->target.pass);
            }
        }
    }
}

static void 
        swap_elem_priority_queue(GtkRotCtrl* ctrl, int i, int j)
{
    /* Finish CODE here */
}

static int get_elem_index_priority_queue(GtkRotCtrl *ctrl, int i){
    int j = ctrl->target.sats[i];

    if(j == NOT_IN_PRIORITY_QUEUE){
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Error in priority list"),
                     __FILE__, __LINE__);
    }
    return j;
}

static void
        next_elem_to_track(GtkRotCtrl* ctrl)
{
    int i, num_sats;
    gdouble min_el, aos_old, aos, los;
    sat_t *sat_old, *sat_new;
    pass_t *pass;
    guint dt;

    /* set variables */
    min_el = sat_cfg_get_int (SAT_CFG_INT_PRED_MIN_EL);
    num_sats = ctrl->target.numSatToTrack;

    if(num_sats <= 0)
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Error tracking new satellite"),
                     __FILE__, __LINE__);

    /* Gets the first satellite of the priority queue */
    sat_old = SAT (g_slist_nth_data(ctrl->sats, ctrl->target.priorityQueue[0]));
    if(sat_old->el > 0.0f) {
        pass = get_current_pass(sat_old, ctrl->qth, ctrl->t);
        aos_old = pass->aos;
        free_pass(pass);
    }
    else {
        aos_old = sat_old->aos;
    } 

    for(i=1; i < num_sats; i++){
        /* Gets the next satellite in the priority queue */
        sat_new = SAT (g_slist_nth_data(ctrl->sats, ctrl->target.priorityQueue[i]));
        if(sat_new->el > 0.0f) {
            pass = get_current_pass(sat_new, ctrl->qth, ctrl->t);
            aos = pass->aos;
            los = pass->los;
            free_pass(pass);
        }
        else {
            aos = sat_new->aos;
            los = sat_new->los;
        }

        /* convert julian date to seconds */
        dt = (guint) ((los - aos) * 86400);

        /* if we have the minimal communication time needded */
        if(dt > ctrl->target.minCommunication[ctrl->target.priorityQueue[i]]){
            /* if the new satellite will set before the rise of the old one */
            if(aos_old > los){
                sat_old = sat_new;
                aos_old = aos;
            }
        }
    }
    ctrl->target.targeting = sat_old;
    if(sat_old->el > 0.0f) 
        ctrl->target.pass = get_current_pass(sat_old, ctrl->qth, ctrl->t);
    else 
        ctrl->target.pass = get_pass(sat_old, ctrl->qth, ctrl->t, 3.0f);
     

    set_flipped_pass(ctrl);
}

static void
        update_tracked_elem(GtkRotCtrl * ctrl)
{
    if(ctrl->target.targeting != NULL){
        next_elem_to_track(ctrl);
        gtk_label_set_text(ctrl->track_sat, ctrl->target.targeting->nickname);
    }
    else{
        gtk_label_set_text(ctrl->track_sat, " --- ");
    }
}
