/***************************************************************************
 *   Copyright (C) 2008 by Alexey Balakin                                  *
 *   mathgl.abalakin@gmail.com                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <QTime>
#include <QMenu>
#include <QPrinter>
#include <QTimer>
#include <QScrollArea>
#include <QPainter>
#include <QPrintDialog>
#include <QToolButton>
#include <QToolBar>
#include <QSpinBox>
#include <QBoxLayout>
#include <QMenuBar>
#include <QMessageBox>
#include <QTextEdit>
#include <QTextBlock>

#include <QMdiArea>
#include "udav_wnd.h"
#include "mgl2/qmathgl.h"
#include "plot_pnl.h"
#include "anim_dlg.h"
#include "style_dlg.h"
#include "newcmd_dlg.h"
#include "subplot_dlg.h"

extern bool mglAutoSave;
extern bool mglHighlight;
extern bool mglDotsRefr;
extern mglParse parser;
int animDelay=500;
void raisePanel(QWidget *w);
//-----------------------------------------------------------------------------
PlotPanel::PlotPanel(QWidget *parent) : QWidget(parent)
{
	gifOn = jpgOn = false;
	animDialog = new AnimParam(this);	animPos = -1;
	stlDialog = new StyleDialog(this);
	newCmdDlg = new NewCmdDialog(this);
	subplotDlg = new SubplotDialog(this);
	printer = new QPrinter;		curPos = subId = -1;
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(next()));
	connect(animDialog, SIGNAL(putText(const QString &)), this, SLOT(animText(const QString &)));
	connect(newCmdDlg, SIGNAL(result(const QString&, bool)), this, SLOT(putCmd(const QString&)));
	connect(subplotDlg, SIGNAL(result(const QString&)), this, SLOT(insCmd(const QString&)));

	menu = new QMenu(tr("Graphics"),this);
	popup = new QMenu(this);
	mgl = new QMathGL(this);
	draw = new mglDrawScript(parser.Self());
	mgl_set_flag(mgl->getGraph(),1,MGL_SHOW_POS);	mgl->setDraw(draw);
	connect(mgl,SIGNAL(askStyle(int)),this,SLOT(setStyle(int)));
	connect(mgl,SIGNAL(objChanged(int)),this,SLOT(setCurPos(int)));

	QBoxLayout *v = new QVBoxLayout(this);	toolTop(v);
	QBoxLayout *h = new QHBoxLayout();	v->addLayout(h);	toolLeft(h);
	mgl->setPopup(popup);

	sv = new QScrollArea(this);	h->addWidget(sv);	sv->setWidget(mgl);
	emit giveFocus();
}
//-----------------------------------------------------------------------------
PlotPanel::~PlotPanel()	{	delete printer;	}
//-----------------------------------------------------------------------------
void PlotPanel::setStyle(int id)
{	if(stlDialog->exec())	mgl->setStyle(id, stlDialog->getStyle());	}
//-----------------------------------------------------------------------------
void PlotPanel::setSubId(int id)	{	subId = id;	}
//-----------------------------------------------------------------------------
void PlotPanel::animText(const QString &txt)	{	animPutText(txt);	}
//-----------------------------------------------------------------------------
void PlotPanel::setCurPos(int pos)
{
	if(!mglHighlight)	pos = -1;
	if(curPos!=pos)	{	curPos = pos;	execute();	}
}
//-----------------------------------------------------------------------------
void PlotPanel::stop()	{	parser.Stop();	mgl->stop();	}
//-----------------------------------------------------------------------------
void PlotPanel::execute()
{
	if(mglAutoSave)	save();
	mgl->setDotsPreview(mglDotsRefr);
	raisePanel(this);
	objId = subId = -1;
	emit clearWarn();
	QTime t;	t.start();
	mgl_set_facenum(mgl->getGraph(),0);
	draw->text=textMGL->toPlainText();
	draw->line=curPos;
	mgl->update();
	setStatus(QString(tr("Drawing time %1 ms")).arg(t.elapsed()*1e-3));
	emit giveFocus();
}
//-----------------------------------------------------------------------------
void PlotPanel::pressF9()
{
	int l=animParam.length(), i;
	wchar_t *str = new wchar_t[l+2];
	animPos = 0;	curPos = -1;
	QString cur = animParam.section('\n',animPos,animPos);
	for(i=0;i<l;i++)	str[i] = (cur[i]).unicode();
	str[i] = 0;
	parser.AddParam(0,str);
	delete []str;

	QTime t;	t.start();
	parser.RestoreOnce();
	draw->text=textMGL->toPlainText();
	draw->line=curPos;	mgl->update();
	setStatus(QString(tr("Drawing time %1 ms")).arg(t.elapsed()*1e-3));
	emit giveFocus();
}
//-----------------------------------------------------------------------------
void PlotPanel::animStart(bool st)
{
	if(!st)
	{	timer->stop();	if(gifOn)	mgl_close_gif(mgl->getGraph());	return;	}
	if(animParam.isEmpty())
	{
		if(animDialog->exec())
		{
			animParam = animDialog->getResult();
			gifOn = animDialog->gifOn;
			jpgOn = animDialog->jpgOn;
		}
		else	return;
	}
	timer->start(animDelay);
	if(gifOn)
	{
		mglGraph gr(mgl->getGraph());
		gr.StartGIF("", animDelay);
		gr.ResetFrames();
	}
	raisePanel(this);
}
//-----------------------------------------------------------------------------
void PlotPanel::nextSlide()
{
	animSwitch(false);
	next();
	emit giveFocus();
}
//-----------------------------------------------------------------------------
void PlotPanel::next()
{
	if(animParam.isEmpty())
	{
		if(animDialog->exec())
		{
			animParam = animDialog->getResult();
			gifOn = animDialog->gifOn;
			jpgOn = animDialog->jpgOn;
		}
		else	return;
	}
	int l=animParam.length(), n=animParam.count('\n') + (animParam[l-1]=='\n' ? 0:1), i;
	wchar_t *str = new wchar_t[l+2];
	animPos = (animPos+1)%n;
	QString cur = animParam.section('\n',animPos,animPos);
	for(i=0;i<l;i++)	str[i] = (cur[i]).unicode();
	str[i] = 0;
	parser.AddParam(0,str);
	delete []str;
	mglGraph gr(mgl->getGraph());
	if(gr.GetNumFrame() >= n)	execute();
	else
	{
		gr.NewFrame();	execute();	gr.EndFrame();
		if(jpgOn)	gr.WriteFrame();
		QString s;	s.sprintf("%d - %d of %d",gr.GetNumFrame(),animPos,n);
		setStatus(QString(tr("Frame %1 of %2")).arg(animPos).arg(n));
	}
}
//-----------------------------------------------------------------------------
void PlotPanel::prevSlide()
{
	if(animParam.isEmpty())
	{
		if(animDialog->exec())
		{
			animParam = animDialog->getResult();
			gifOn = animDialog->gifOn;
			jpgOn = animDialog->jpgOn;
		}
		else	return;
	}
	animSwitch(false);
	int l=animParam.length(), n=animParam.count('\n') + (animParam[l-1]=='\n' ? 0:1), i;
	wchar_t *str = new wchar_t[l+2];
	animPos = (animPos-1+n)%n;
	QString cur = animParam.section('\n',animPos,animPos);
	for(i=0;i<l;i++)	str[i] = (cur[i]).unicode();
	str[i] = 0;
	parser.AddParam(0,str);
	delete []str;
	execute();
	emit giveFocus();
}
//-----------------------------------------------------------------------------
void PlotPanel::animSetup()
{
	if(animDialog->exec())
	{
		animParam = animDialog->getResult();
		gifOn = animDialog->gifOn;
		jpgOn = animDialog->jpgOn;
		animPos = -1;
	}
}
//-----------------------------------------------------------------------------
void PlotPanel::adjust()
{
	mgl->setSize(sv->width()-5, sv->height()-5);
	raisePanel(this);	emit giveFocus();
}
//-----------------------------------------------------------------------------
void PlotPanel::setMGLFont(const QString &path)	{	mgl->setMGLFont(path);	}
//-----------------------------------------------------------------------------
void PlotPanel::animParseText(const QString &txt)
{
	int i, n = txt.count('\n')+1;
	double a1=0,a2=0,da=0;
	QString s, all;
	for(i=0;i<n;i++)
	{
		s = txt.section('\n',i,i);
		if(s[0]=='#' && s[1]=='#' && s[2]=='a' && s[3].isSpace())
			all = all + s.mid(4) + "\n";
		if(s[0]=='#' && s[1]=='#' && s[2]=='c' && s[3].isSpace())
		{
			s = s.mid(4);
			a1 = s.section(' ',0,0).toDouble();
			a2 = s.section(' ',1,1).toDouble();
			da = s.section(' ',2,2).toDouble();
			animDialog->setResult(a1,a2,da);
		}
	}
	if(!all.isEmpty())
	{
		animDialog->setResult(all);
		animParam = all;
	}
	else if(a1!=a2 && da!=0)
	{
		for(double a=a1;a<=a2;a+=da)	all = all + QString::number(a)+"\n";
		animParam = all;
	}
}
//-----------------------------------------------------------------------------
//#include "xpm/wire.xpm"
#include "xpm/text.xpm"
#include "xpm/line.xpm"
#include "xpm/curve.xpm"
#include "xpm/mark_o.xpm"
#include "xpm/mark_s.xpm"
#include "xpm/mark_a.xpm"
#include "xpm/mark_d.xpm"
#include "xpm/arc.xpm"
#include "xpm/polygon.xpm"
#include "xpm/box.xpm"
//-----------------------------------------------------------------------------
void PlotPanel::toolTop(QBoxLayout *v)
{
	QAction *a, *aa;
	QMenu *o=menu, *oo;
	QToolBar *t = new QToolBar(this);	v->addWidget(t);	t->setMovable(false);

	// graphics menu
	a = new QAction(QPixmap(":/png/alpha.png"), tr("Alpha"), this);
	a->setShortcut(Qt::CTRL+Qt::Key_T);	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), mgl, SLOT(setAlpha(bool)));
	connect(mgl, SIGNAL(alphaChanged(bool)), a, SLOT(setChecked(bool)));
	a->setToolTip(tr("Switch on/off transparency for the graphics (Ctrl+T)."));
	o->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/weather-clear.png"), tr("Light"), this);
	a->setShortcut(Qt::CTRL+Qt::Key_L);	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), mgl, SLOT(setLight(bool)));
	connect(mgl, SIGNAL(lightChanged(bool)), a, SLOT(setChecked(bool)));
	a->setToolTip(tr("Switch on/off lightning for the graphics (Ctrl+L)."));
	o->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/view-grid.png"), tr("Grid"), this);
	a->setShortcut(Qt::CTRL+Qt::Key_G);	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), mgl, SLOT(setGrid(bool)));
	a->setToolTip(tr("Switch on/off grid of absolute coordinates (Ctrl+G)."));
	o->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/transform-move.png"), tr("Rotate by mouse"), this);
	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), mgl, SLOT(setRotate(bool)));
	connect(mgl, SIGNAL(rotateChanged(bool)), a, SLOT(setChecked(bool)));
	a->setToolTip(tr("Switch on/off mouse handling of the graphics\n(rotation, shifting, zooming and perspective)."));
	t->addAction(a);

/*	a = new QAction(QPixmap(":/png/zoom-fit-best.png"), tr("Zoom by mouse"), this);
	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), mgl, SLOT(setZoom(bool)));
	connect(mgl, SIGNAL(zoomChanged(bool)), a, SLOT(setChecked(bool)));
	a->setToolTip(tr("Switch on/off mouse zoom of selected region."));
	t->addAction(a);*/

	o->addSeparator();
	a = new QAction(QPixmap(":/png/zoom-original.png"), tr("Restore"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(restore()));
	a->setToolTip(tr("Restore default graphics rotation, zoom and perspective (Ctrl+Space)."));
	a->setShortcut(Qt::CTRL+Qt::Key_Space);
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/view-refresh.png"), tr("Redraw"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(execute()));
	a->setToolTip(tr("Execute script and redraw graphics (F5)."));
	a->setShortcut(Qt::Key_F5);
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/view-fullscreen.png"), tr("Adjust size"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(adjust()));
	a->setToolTip(tr("Change canvas size to fill whole region (F6)."));
	a->setShortcut(Qt::Key_F6);		o->addAction(a);

	a = new QAction(QPixmap(":/png/document-revert.png"), tr("Reload"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(pressF9()));
	a->setToolTip(tr("Restore status for 'once' command and reload data (F9)."));
	a->setShortcut(Qt::Key_F9);	o->addAction(a);	popup->addAction(a);

	a = new QAction(QPixmap(":/png/process-stop.png"), tr("Stop"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(stop()));
	a->setToolTip(tr("Stop script execution (F7)."));
	a->setShortcut(Qt::Key_F7);	o->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/edit-copy.png"), tr("Copy plot"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(copy()));
	a->setToolTip(tr("Copy graphics to clipboard (Ctrl+Shift+G)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_G);
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/edit-copy.png"), tr("Copy click coor."), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(copyClickCoor()));
	a->setToolTip(tr("Copy coordinates of last mouse click to clipboard."));
	o->addAction(a);	popup->addAction(a);

//	l->addStretch(1);
	{
		oo = new QMenu(tr("Primitives ..."),this);
		aa=a = new QAction(QPixmap(line_xpm), tr("Add line"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addLine()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add line which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(arc_xpm), tr("Add arc"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addArc()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add arc which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(curve_xpm), tr("Add curve"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addCurve()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add curve which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(mark_s_xpm), tr("Add rect"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addRect()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add rectangle which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(mark_d_xpm), tr("Add rhombus"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addRhomb()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add rhombus which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(mark_o_xpm), tr("Add ellipse"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addEllipse()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add ellipse which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(polygon_xpm), tr("Add polygon"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addPolygon()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add ellipse which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(mark_a_xpm), tr("Add mark"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addMark()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add marker which properties can be changed later by mouse."));
		oo->addAction(a);

		a = new QAction(QPixmap(text_xpm), tr("Add text"), this);
		connect(a, SIGNAL(triggered()), mgl, SLOT(addText()));
		connect(mgl, SIGNAL(usePrimChanged(bool)), a, SLOT(setVisible(bool)));
		a->setToolTip(tr("Add text which properties can be changed later by mouse."));
		oo->addAction(a);

		QToolButton *bb = new QToolButton(this);
		bb->setDefaultAction(aa);	bb->setMenu(oo);
		bb->setPopupMode(QToolButton::MenuButtonPopup);
		t->addWidget(bb);
	}

	a = new QAction(QPixmap(":/png/edit-delete.png"), tr("Delete selected"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(deleteSelected()));
	a->setToolTip(tr("Delete selected plot."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/layer-visible-off.png"), tr("Hide selected"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(hideSelected()));
	a->setToolTip(tr("Hide selected plot."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	const MainWindow *mw=findMain(this);
	if(mw)	t->addAction(mw->ahide);

	a = new QAction(QPixmap(":/png/format-indent-more.png"), tr("New command"), this);
	connect(a, SIGNAL(triggered()), newCmdDlg, SLOT(show()));
	a->setToolTip(tr("Show dialog for new command and put it into the script."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(box_xpm), tr("New inplot"), this);
	connect(a, SIGNAL(triggered()), subplotDlg, SLOT(show()));
	a->setToolTip(tr("Show dialog for new inplot and put it into the script."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/object-order-lower.png"), tr("Move plot up"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(movePlotUp()));
	a->setToolTip(tr("Move selected plot up to previous subplot."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/object-order-raise.png"), tr("Move plot down"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(movePlotDown()));
	a->setToolTip(tr("Move selected plot down to next subplot."));
	o->addAction(a);	popup->addAction(a);	t->addAction(a);


	o->addMenu(oo);	//t->addSeparator();

	tet = new QSpinBox(this);	tet->setWrapping(true);
	t->addWidget(tet);	tet->setRange(-180, 180);	tet->setSingleStep(10);
	connect(tet, SIGNAL(valueChanged(int)), mgl, SLOT(setTet(int)));
	connect(mgl, SIGNAL(tetChanged(int)), tet, SLOT(setValue(int)));
	tet->setToolTip(tr("Set value of \\theta angle.\nYou can use keys (Shift+Meta+Up or Shift+Meta+Down)."));

	phi = new QSpinBox(this);	phi->setWrapping(true);
	t->addWidget(phi);	phi->setRange(-180, 180);	phi->setSingleStep(10);
	connect(phi, SIGNAL(valueChanged(int)), mgl, SLOT(setPhi(int)));
	connect(mgl, SIGNAL(phiChanged(int)), phi, SLOT(setValue(int)));
	phi->setToolTip(tr("Set value of \\phi angle.\nYou can use keys (Shift+Meta+Left or Shift+Meta+Right)."));

	oo = new QMenu(tr("Export as 2D ..."),this);
	oo->addAction(tr("PNG"), mgl, SLOT(exportPNG()),Qt::ALT+Qt::Key_P);
	oo->addAction(tr("solid PNG"), mgl, SLOT(exportPNGs()),Qt::ALT+Qt::Key_F);
	oo->addAction(tr("JPEG"), mgl, SLOT(exportJPG()),Qt::ALT+Qt::Key_J);
	oo->addAction(tr("bitmap EPS"), mgl, SLOT(exportBPS()));
	oo->addAction(tr("vector EPS"), mgl, SLOT(exportEPS()),Qt::ALT+Qt::Key_E);
	oo->addAction(tr("SVG"), mgl, SLOT(exportSVG()),Qt::ALT+Qt::Key_S);
	oo->addAction(tr("LaTeX"), mgl, SLOT(exportTEX()),Qt::ALT+Qt::Key_L);

	o->addMenu(oo);		popup->addMenu(oo);
	oo = new QMenu(tr("Export as 3D ..."),this);
	oo->addAction(tr("MGLD"), mgl, SLOT(exportMGLD()),Qt::ALT+Qt::Key_M);
	oo->addAction(tr("PRC"), mgl, SLOT(exportPRC()),Qt::ALT+Qt::Key_D);
	oo->addAction(tr("OBJ"), mgl, SLOT(exportOBJ()),Qt::ALT+Qt::Key_O);
	oo->addAction(tr("STL"), mgl, SLOT(exportSTL()));
	oo->addAction(tr("XYZ"), mgl, SLOT(exportXYZ()));
//	oo->addAction(tr("X3D"), QMGL, SLOT(exportX3D()),Qt::ALT+Qt::Key_X);	// TODO: Add later
	o->addMenu(oo);		popup->addMenu(oo);
}
//-----------------------------------------------------------------------------
void PlotPanel::toolLeft(QBoxLayout *h)
{
	QAction *a;
	QMenu *o=menu, *oo;
	QToolBar *t = new QToolBar(this);	h->addWidget(t);	t->setMovable(false);
	t->setAllowedAreas(Qt::LeftToolBarArea);	t->setOrientation(Qt::Vertical);

	// zooming menu
	oo = o->addMenu(tr("Zoom/move"));
	a = new QAction(QPixmap(":/png/arrow-left.png"), tr("Move left"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(shiftLeft()));
	a->setShortcut(Qt::ALT+Qt::Key_Left);
	a->setToolTip(tr("Move graphics left by 1/3 of its width."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/arrow-up.png"), tr("Move up"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(shiftUp()));
	a->setShortcut(Qt::ALT+Qt::Key_Up);
	a->setToolTip(tr("Move graphics up by 1/3 of its height."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/zoom-in.png"), tr("Zoom in"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(zoomIn()));
	a->setShortcut(Qt::ALT+Qt::Key_Equal);
	a->setToolTip(tr("Zoom in graphics."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/zoom-out.png"), tr("Zoom out"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(zoomOut()));
	a->setShortcut(Qt::ALT+Qt::Key_Minus);
	a->setToolTip(tr("Zoom out graphics."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/arrow-down.png"), tr("Move down"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(shiftDown()));
	a->setShortcut(Qt::ALT+Qt::Key_Down);
	a->setToolTip(tr("Move graphics up down 1/3 of its height."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/arrow-right.png"), tr("Move right"), this);
	connect(a, SIGNAL(triggered()), mgl, SLOT(shiftRight()));
	a->setShortcut(Qt::ALT+Qt::Key_Right);
	a->setToolTip(tr("Move graphics right by 1/3 of its width."));
	oo->addAction(a);	t->addAction(a);

	// rotate menu
	oo = o->addMenu(tr("Rotate"));
	a = new QAction(QPixmap(":/png/object-rotate-up.png"), tr("Rotate up"), this);
	a->setShortcut(Qt::SHIFT+Qt::META+Qt::Key_Up);
	connect(a, SIGNAL(triggered()), tet, SLOT(stepUp()));	oo->addAction(a);
	a->setToolTip(tr("Increase \\theta angle by 10 degrees."));
	a = new QAction(QPixmap(":/png/object-rotate-down.png"), tr("Rotate down"), this);
	a->setShortcut(Qt::SHIFT+Qt::META+Qt::Key_Down);
	connect(a, SIGNAL(triggered()), tet, SLOT(stepDown()));	oo->addAction(a);
	a->setToolTip(tr("Decrease \\theta angle by 10 degrees."));
	a = new QAction(QPixmap(":/png/object-rotate-left.png"), tr("Rotate left"), this);
	a->setShortcut(Qt::SHIFT+Qt::META+Qt::Key_Right);
	connect(a, SIGNAL(triggered()), phi, SLOT(stepUp()));	oo->addAction(a);
	a->setToolTip(tr("Increase \\phi angle by 10 degrees."));
	a = new QAction(QPixmap(":/png/object-rotate-right.png"), tr("Rotate right"), this);
	a->setShortcut(Qt::SHIFT+Qt::META+Qt::Key_Left);
	connect(a, SIGNAL(triggered()), phi, SLOT(stepDown()));	oo->addAction(a);
	a->setToolTip(tr("Decrease \\phi angle by 10 degrees."));

	// animation menu
	oo = o->addMenu(tr("Animation"));
	a = new QAction(QPixmap(":/png/media-seek-forward.png"), tr("Next slide"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(nextSlide()));
	a->setShortcut(Qt::CTRL+Qt::Key_Period);
	a->setToolTip(tr("Show next slide (Ctrl+.)."));
	oo->addAction(a);	t->addAction(a);

	a = new QAction(QPixmap(":/png/media-playback-start.png"), tr("Slideshow"), this);
	a->setCheckable(true);
	connect(a, SIGNAL(toggled(bool)), this, SLOT(animStart(bool)));
	connect(this, SIGNAL(animSwitch(bool)),a,SLOT(setChecked(bool)));
	a->setToolTip(tr("Run slideshow (Ctrl+F5). If no parameter specified\nthen the dialog with slideshow options will appear."));
	a->setShortcut(Qt::CTRL+Qt::Key_F5);	oo->addAction(a);	t->addAction(a);

	oo->addAction(tr("Setup show"), this, SLOT(animSetup()), Qt::CTRL+Qt::Key_W);

	a = new QAction(QPixmap(":/png/media-seek-backward.png"), tr("Prev slide"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(prevSlide()));
	a->setShortcut(Qt::CTRL+Qt::Key_Comma);
	a->setToolTip(tr("Show previous slide (Ctrl+,)."));
	oo->addAction(a);	t->addAction(a);
}
//-----------------------------------------------------------------------------
QString PlotPanel::getFit()	{	return QString(mgl_get_fit(mgl->getGraph()));	}
//-----------------------------------------------------------------------------
void PlotPanel::deleteSelected()
{
	if(curPos>=0)
	{
		textMGL->moveCursor(QTextCursor::Start);
		for(int i=0;i<curPos;i++)	textMGL->moveCursor(QTextCursor::NextBlock);
		QTextCursor tc= textMGL->textCursor();
		tc.select(QTextCursor::LineUnderCursor);
		tc.removeSelectedText();
		tc.deleteChar();
		curPos = -1;	execute();
	}
	else emit setStatus("No selection.");
}
//-----------------------------------------------------------------------------
void PlotPanel::hideSelected()
{
	if(curPos>=0)
	{
		textMGL->moveCursor(QTextCursor::Start);
		for(int i=0;i<curPos;i++)	textMGL->moveCursor(QTextCursor::NextBlock);
		textMGL->insertPlainText("#h ");
		curPos = -1;	execute();
	}
	else emit setStatus("No selection.");
}
//-----------------------------------------------------------------------------
void PlotPanel::putCmd(const QString &cmd)
{
	textMGL->moveCursor(QTextCursor::Start);
	if(curPos>=0)	for(int i=0;i<curPos;i++)
		textMGL->moveCursor(QTextCursor::NextBlock);
	textMGL->insertPlainText(cmd+"\n");
	curPos = -1;	execute();
}
//-----------------------------------------------------------------------------
void PlotPanel::insCmd(const QString &cmd)
{
	textMGL->moveCursor(QTextCursor::EndOfBlock);
	textMGL->insertPlainText("\n"+cmd);
	curPos = -1;	execute();
}
//-----------------------------------------------------------------------------
void PlotPanel::movePlotUp()
{
	if(curPos>0)
	{
		QTextCursor tc = textMGL->textCursor();
		tc.movePosition(QTextCursor::Start);
		tc.movePosition(QTextCursor::NextBlock,QTextCursor::MoveAnchor,curPos);
		tc.select(QTextCursor::BlockUnderCursor);
		QString s = tc.selectedText();
		tc.deleteChar();
		bool ins=true;

		while(tc.movePosition(QTextCursor::PreviousBlock))
		{
			QString q = tc.block().text();
			if(q.startsWith("subplot ") || q.startsWith("inplot ") || q.startsWith("multiplot ") || q.startsWith("gridplot ") || q.startsWith("columnplot ") || q.startsWith("stickplot "))
			{
				tc.movePosition(QTextCursor::EndOfBlock);
				tc.insertText(s);	ins=false;	break;
			}
		}
		if(ins)
		{
			tc.movePosition(QTextCursor::Start);	tc.insertText(s+"\n");
			tc.movePosition(QTextCursor::Start);	tc.deleteChar();
		}
		curPos = tc.block().blockNumber();	execute();
	}
	else if(curPos<0)	emit setStatus("No selection.");
}
//-----------------------------------------------------------------------------
void PlotPanel::movePlotDown()
{
	if(curPos>=0)
	{
		QTextCursor tc = textMGL->textCursor();
		tc.movePosition(QTextCursor::Start);
		tc.movePosition(QTextCursor::NextBlock,QTextCursor::MoveAnchor,curPos);
		tc.select(QTextCursor::BlockUnderCursor);
		QString s = tc.selectedText();
		if(curPos==0)	s = "\n"+s;
		tc.deleteChar();
		bool ins=true;

		while(tc.movePosition(QTextCursor::NextBlock))
		{
			QString q = tc.block().text();
			if(q.startsWith("subplot ") || q.startsWith("inplot ") || q.startsWith("multiplot ") || q.startsWith("gridplot ") || q.startsWith("columnplot ") || q.startsWith("stickplot "))
			{
				tc.movePosition(QTextCursor::EndOfBlock);
				tc.insertText(s);	ins=false;	break;
			}
		}
		if(ins)	{	tc.movePosition(QTextCursor::End);	tc.insertText(s);	}
		curPos = tc.block().blockNumber();	execute();
	}
	else emit setStatus("No selection.");
}
//-----------------------------------------------------------------------------

