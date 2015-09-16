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
#include <QTableWidget>
#include <QLabel>
#include <QAction>
#include <QLayout>
#include <QMenuBar>
#include <QLineEdit>
#include <QMenu>
#include <QClipboard>
#include <QStatusBar>
#include <QFileDialog>
#include <QPushButton>
#include <QApplication>
#include <QInputDialog>
#include <QToolButton>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QMessageBox>
#include <mgl2/mgl.h>
//-----------------------------------------------------------------------------
#include "dat_pnl.h"
#include "info_dlg.h"
#include "xpm/table.xpm"
//-----------------------------------------------------------------------------
extern mglParse parser;
void updateDataItems();
void addDataPanel(QWidget *wnd, QWidget *w, QString name);
void deleteDat(void *o)		{	if(o)	delete ((DatPanel *)o);	}
void refreshData(QWidget *w)	{	((DatPanel *)w)->refresh();	}
//-----------------------------------------------------------------------------
QWidget *newDataWnd(InfoDialog *inf, QWidget *wnd, mglDataA *v)
{
	DatPanel *t = new DatPanel(inf);
	if(v)	t->setVar(v);
	addDataPanel(wnd,t,t->dataName());
	return t;
}
//-----------------------------------------------------------------------------
DatPanel::DatPanel(InfoDialog *inf, QWidget *parent) : QWidget(parent)
{
	setAttribute(Qt::WA_DeleteOnClose);
	kz = nx = ny = nz = 0;	var = 0;
	ready = false;	infoDlg = inf;
	QBoxLayout *v,*h,*m;

	menu = new QMenu(tr("Data"),this);
	v = new QVBoxLayout(this);
	h = new QHBoxLayout();	v->addLayout(h);	toolTop(h);
	h = new QHBoxLayout();	v->addLayout(h);
	m = new QVBoxLayout();	h->addLayout(m);	toolLeft(m);
	tab = new QTableWidget(this);	h->addWidget(tab);
	connect(tab, SIGNAL(cellChanged(int,int)), this, SLOT(putValue(int, int)));

	setWindowIcon(QPixmap(table_xpm));
}
//-----------------------------------------------------------------------------
DatPanel::~DatPanel()	{	if(var && var->o==this)	var->o = 0;	}
//-----------------------------------------------------------------------------
void DatPanel::refresh()
{
	bool rc = false;
	if(!var)	return;
	infoDlg->allowRefresh=false;
	if(nx!=var->GetNx())	{	nx = var->GetNx();	tab->setColumnCount(nx);	rc=true;	}
	if(ny!=var->GetNy())	{	ny = var->GetNy();	tab->setRowCount(ny);	rc=true;	}
	if(kz>=var->GetNz())	{	kz = 0;	emit sliceChanged(0);	}
	if(nz!=var->GetNz())	{	nz = var->GetNz();	emit nzChanged(nz);		}
	const mglData *dd = dynamic_cast<const mglData *>(var);	if(dd)	id = QString(dd->id.c_str());
	const mglDataC *dc = dynamic_cast<const mglDataC *>(var);	if(dc)	id = QString(dc->id.c_str());
	if(nz==1 && ny>1 && !id.isEmpty())
	{
		QStringList head;
		QString s;
		for(int i=0;i<ny;i++)
		{
			s = QString("%1").arg(i);
			if(id[i]>='a' && id[i]<='z')	s=s+" ("+id[i]+")";
			head<<s;
		}
		tab->setHorizontalHeaderLabels(head);
	}
	long m=var->s.length();
	QString s,d;
	if(rc)
	{
		QStringList sh,sv;
		for(long i=0;i<nx;i++)	sh<<QString::number(i);
		tab->setHorizontalHeaderLabels(sh);
		for(long i=0;i<ny;i++)	sv<<QString::number(i);
		tab->setVerticalHeaderLabels(sv);
		for(long i=0;i<nx;i++)	for(long j=0;j<ny;j++)
			tab->setItem(j,i,new QTableWidgetItem);
	}
	mglDataC *cc = dynamic_cast<mglDataC*>(var);
	if(cc)	for(long i=0;i<nx;i++)	for(long j=0;j<ny;j++)
	{
		dual f = cc->a[i+nx*(j+ny*kz)];
		if(mgl_isnan(f))	s = "nan";
		else if(mgl_isbad(f))	s="inf";
		else if(imag(f)>0)	s.sprintf("%.15g+%.15gi",real(f),imag(f));
		else if(imag(f)<0)	s.sprintf("%.15g-%.15gi",real(f),-imag(f));
		else	s.sprintf("%15g",real(f));
		tab->item(j,i)->setText(s);
	}
	else	for(long i=0;i<nx;i++)	for(long j=0;j<ny;j++)
	{
		double f = var->v(i,j,kz);
		if(mgl_isnan(f))	s = "nan";
		else if(mgl_isbad(f))	s=f>0?"inf":"-inf";
		else	s.sprintf("%.15g",f);
		tab->item(j,i)->setText(s);
	}
	infoDlg->allowRefresh=true;	infoDlg->refresh();
	QChar *ss = new QChar[m+1];
	for(long i=0;i<m;i++)	ss[i] = var->s[i];
	s = QString(ss, m);	delete []ss;
	d.sprintf("%d * %d * %d", nx, ny, nz);
	ready = true;
}
//-----------------------------------------------------------------------------
void DatPanel::setVar(mglDataA *v)
{
	ready = false;
	if(var)	var->o = 0;
	var = v;	infoDlg->setVar(v);
	nx = ny = nz = kz = 0;
	if(v)
	{
		QString s = QString::fromStdWString(v->s);
		v->o = this;	v->func = deleteDat;
		refresh();
		setWindowTitle(s + tr(" - UDAV variable"));
		infoDlg->setWindowTitle(s + tr(" - UDAV preview"));
	}
	else
	{	tab->setColumnCount(0);	tab->setRowCount(0);	emit nzChanged(nz);	}
	emit sliceChanged(0);
}
//-----------------------------------------------------------------------------
void DatPanel::setSlice(int k)
{
	if(k>=nz)	k=nz-1;	if(k<0)	k=0;
	if(k!=kz)
	{
		infoDlg->setSlice(k);
		emit sliceChanged(k);
		kz = k;		refresh();
	}
}
//-----------------------------------------------------------------------------
dual mgl_str2dual(const char *s)
{
	setlocale(LC_NUMERIC, "C");
	double re=0,im=0;	size_t ll=strlen(s);
	while(s[ll]<=' ')	ll--;
	if(*s=='(')		sscanf(s,"(%lg,%lg)",&re,&im);
	else if(*s=='i')	{	re=0;	im=atof(s+1);	}
	else if(*s=='[')	sscanf(s,"[%lg,%lg]",&re,&im);
	else if(*s=='{')	sscanf(s,"{%lg,%lg}",&re,&im);
	else if(s[ll]=='i')
	{
		double a,b;
		int s1=sscanf(s,"%lg+%lgi",&re,&im);
		int s2=sscanf(s,"%lg-%lgi",&a,&b);
		if(s1<2)
		{
		if(s2==2)	{	re=a;	im=-b;	}
		else	{	im=atof(s);	re=0;	}
		}
	}
	else
	{
		double a,b;
		int s1=sscanf(s,"%lg+i%lg",&re,&im);
		int s2=sscanf(s,"%lg-i%lg",&a,&b);
		if(s1<2)
		{
		if(s2==2)	{	re=a;	im=-b;	}
		else	{	re=atof(s);	im=0;	}
		}
	}
	setlocale(LC_NUMERIC, "");
	return dual(re,im);
}
//-----------------------------------------------------------------------------
void DatPanel::putValue(int r, int c)
{
	if(!var || r<0 || c<0 || r>=ny || c>=nx || !ready)	return;
	QString s = tab->item(r,c)->text().toLower();
	mreal f;
	dual g;
	if(s=="nan")	f=NAN;
	else if(s=="inf")	f=INFINITY;
	else if(s=="-inf")	f=-INFINITY;
	else	g = mgl_str2dual(s.toStdString().c_str());	//f = s.toDouble();
	f = real(g);
	mglDataC *cc = dynamic_cast<mglDataC*>(var);
	if(cc)
	{
		if(g!=cc->a[c+nx*(r+ny*kz)])
		{
			if(mgl_isnan(g))	s="nan";
			else if(mgl_isbad(g))	s="inf";
			else if(imag(g)>0)	s.sprintf("%g+%gi",real(g),imag(g));
			else if(imag(g)<0)	s.sprintf("%g-%gi",real(g),-imag(g));
			else	s.sprintf("%g",real(g));
			tab->item(r,c)->setText(s);
		}
		cc->a[c+nx*(r+ny*kz)] = g;
	}
	else
	{
		if(f!=var->v(c,r,kz))
		{
			if(mgl_isnan(f))	s="nan";
			else if(mgl_isbad(f))	s=f>0?"inf":"-inf";
			else	s.sprintf("%g", f);
			tab->item(r,c)->setText(s);
		}
		var->set_v(f,c,r,kz);
	}
	infoDlg->refresh();
}
//-----------------------------------------------------------------------------
void DatPanel::save()
{
	QString fn = QFileDialog::getSaveFileName(this, tr("UDAV - Save/export data"), "",
				tr("Data files (*.dat)\nHDF5 files (*.h5 *.hdf)\nPNG files (*.png)\nAll files (*.*)"));
	if(fn.isEmpty())	return;
	QString ext = fn.section(".",-1);
	if(ext=="png")
	{
		bool ok;
		QString s = QInputDialog::getText(this, tr("UDAV - Export to PNG"), tr("Enter color scheme for picture"), QLineEdit::Normal, MGL_DEF_SCH, &ok);
		if(ok)	var->Export(fn.toStdString().c_str(), s.toStdString().c_str());
	}
	else if(ext=="h5" || ext=="hdf")
	{
		bool ok;
		QString s = QInputDialog::getText(this, tr("UDAV - Save to HDF"), tr("Enter data name"), QLineEdit::Normal, QString::fromStdWString(var->s), &ok);
		if(ok)	var->SaveHDF(fn.toStdString().c_str(), s.toStdString().c_str());
	}
	else 	var->Save(fn.toStdString().c_str());
}
//-----------------------------------------------------------------------------
void DatPanel::load()
{
	mglData *d = dynamic_cast<mglData *>(var);	if(!d)	return;
	QString fn = QFileDialog::getOpenFileName(this, tr("UDAV - Load data"), "",
				tr("Data files (*.dat)\nHDF5 files (*.h5 *.hdf)\nPNG files (*.png)\nAll files (*.*)"));
	if(fn.isEmpty())	return;
	QString ext = fn.section(".",-1);
	if(ext=="png")
	{
		bool ok;
		QString s = QInputDialog::getText(this, tr("UDAV - Import PNG"), tr("Enter color scheme for picture"), QLineEdit::Normal, MGL_DEF_SCH, &ok);
		if(ok)	d->Import(fn.toStdString().c_str(), s.toStdString().c_str());
	}
	else if(ext=="h5" || ext=="hdf")
	{
		bool ok;
		QString s = QInputDialog::getText(this, tr("UDAV - Read from HDF"), tr("Enter data name"), QLineEdit::Normal, QString::fromStdWString(var->s), &ok);
		if(ok)	d->ReadHDF(fn.toStdString().c_str(), s.toStdString().c_str());
	}
	else 	d->Read(fn.toStdString().c_str());
	refresh();
}
//-----------------------------------------------------------------------------
void DatPanel::copy()
{
	QTableWidgetSelectionRange ts = tab->selectedRanges().first();
	register long i,j;
	QString res, s;
	for(j=ts.topRow();j<=ts.bottomRow();j++)
	{
		for(i=ts.leftColumn();i<=ts.rightColumn();i++)
		{
			res = res + tab->item(j,i)->text();
			if(i<ts.rightColumn())	res = res + "\t";
		}
		res = res + "\n";
	}
	QApplication::clipboard()->setText(res, QClipboard::Clipboard);
}
//-----------------------------------------------------------------------------
void DatPanel::paste()
{
	QString txt = QApplication::clipboard()->text(QClipboard::Clipboard);
	QString s, t;
	int r = tab->currentRow(), c = tab->currentColumn(), i, j;
	for(i=0;i<ny-r;i++)
	{
		s = txt.section('\n',i,i,QString::SectionSkipEmpty);
		if(s.isEmpty())	break;
		for(j=0;j<nx-c;j++)
		{
			t = s.section('\t',j,j,QString::SectionSkipEmpty);
			if(t.isEmpty())	{	j=nx;	continue;	}
			var->set_v(t.toDouble(),j+c,i+r,kz);
		}
	}
	refresh();
}
//-----------------------------------------------------------------------------
void DatPanel::plot()	// TODO: plot dialog
{

}
//-----------------------------------------------------------------------------
void DatPanel::list()	// TODO: in which script insert ???
{
/*	if(nx*ny+ny > 1020)
	{	QMessageBox::warning(this, tr("UDAV - To list conversion"), tr("Too many numbers (>1000) on slice"), QMessageBox::Ok, 0, 0);	return;	}
	if(nz > 1)
		QMessageBox::information(this, tr("UDAV - To list conversion"), tr("Only current slice will be inserted"), QMessageBox::Ok, 0, 0);
	QString res = "list\t", s;
	register long i,j;
	for(j=0;j<ny;j++)
	{
		for(i=0;i<nx;i++)
		{
			s.sprintf("%g\t",d->a[i+nx*(j+kz*ny)]);
			res += s;
		}
		if(j<ny-1)	res = res + "|\t";
	}*/
}
//-----------------------------------------------------------------------------
void DatPanel::inrange()
{
	QString v1("-1"), v2("1"), dir("x");
	if(sizesDialog(tr("UDAV - Fill data"), tr("Enter range for data and direction of filling"), tr("From"), tr("To"), tr("Direction"), v1, v2, dir))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Fill(v1.toDouble(), v2.toDouble(), dir[0].toLatin1());
		mglDataC *dc = dynamic_cast<mglDataC *>(var);
		if(dc)	dc->Fill(v1.toDouble(), v2.toDouble(), dir[0].toLatin1());
		mglDataV *dv = dynamic_cast<mglDataV *>(var);
		if(dv)	dv->Fill(v1.toDouble(), v2.toDouble(), dir[0].toLatin1());
		refresh();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::norm()
{
	QString v1("0"), v2("1"), how;
	if(sizesDialog(tr("UDAV - Normalize data"), tr("Enter range for final data"), tr("From"), tr("To"), tr("Symmetrical?"), v1, v2, how))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Norm(v1.toDouble(), v2.toDouble(), (how=="on" || how.contains('s')));
		refresh();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::normsl()
{
	QString v1("0"), v2("1"), dir("z");
	if(sizesDialog(tr("UDAV - Normalize by slice"), tr("Enter range for final data"), tr("From"), tr("To"), tr("Direction"), v1, v2, dir))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->NormSl(v1.toDouble(), v2.toDouble(), dir[0].toLatin1());
		refresh();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::create()
{
	QString mx, my("1"), mz("1");
	if(sizesDialog(tr("UDAV - Clear data"), tr("Enter new data sizes"), tr("X-size"), tr("Y-size"), tr("Z-size"), mx, my, mz))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Create(mx.toInt(), my.toInt(), mz.toInt());
		mglDataC *c = dynamic_cast<mglDataC *>(var);
		if(c)	c->Create(mx.toInt(), my.toInt(), mz.toInt());
		refresh();	updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::reSize()
{
	QString mx, my, mz;
	mx.sprintf("%d",nx);	my.sprintf("%d",ny);	mz.sprintf("%d",nz);
	if(sizesDialog(tr("UDAV - Resize data"), tr("Enter new data sizes"), tr("X-size"), tr("Y-size"), tr("Z-size"), mx, my, mz))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Set(d->Resize(mx.toInt(), my.toInt(), mz.toInt()));
		refresh();	updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::squize()
{
	QString mx("1"), my("1"), mz("1");
	if(sizesDialog(tr("UDAV - Squeeze data"), tr("Enter step of saved points. For example, '1' save all, '2' save each 2nd point, '3' save each 3d and so on."), tr("X-direction"), tr("Y-direction"), tr("Z-direction"), mx, my, mz))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Squeeze(mx.toInt(), my.toInt(), mz.toInt());
		refresh();	updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::crop()
{
	QString n1("1"), n2("1"), dir;
	if(sizesDialog(tr("UDAV - Crop data"), tr("Enter range of saved date."), tr("From"), tr("To"), tr("Direction"), n1, n2, dir))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Squeeze(n1.toInt(), n2.toInt(), dir[0].toLatin1());
		refresh();	updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::rearrange()
{
	QString mx, my, mz;
	mx.sprintf("%d",nx);	my.sprintf("%d",ny);	mz.sprintf("%d",nz);
	if(sizesDialog(tr("UDAV - Rearrange data"), tr("Enter new data sizes"), tr("X-size"), tr("Y-size"), tr("Z-size"), mx, my, mz))
	{
		mglData *d = dynamic_cast<mglData *>(var);
		if(d)	d->Rearrange(mx.toInt(), my.toInt(), mz.toInt());
		refresh();	updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::hist()
{
	QLabel *l;
	QLineEdit *id, *v1, *v2;
	QSpinBox *nm;
	QPushButton *b;
	QDialog *d = new QDialog(this);	d->setWindowTitle(tr("UDAV - Make histogram"));
	QGridLayout *g = new QGridLayout(d);
	l = new QLabel(tr("From"), d);	g->addWidget(l,0,0);
	l = new QLabel(tr("To"), d);	g->addWidget(l,0,1);
	v1 = new QLineEdit(d);	g->addWidget(v1,1,0);
	v2 = new QLineEdit(d);	g->addWidget(v2,1,1);
	l = new QLabel(tr("Number of points"), d);	g->addWidget(l,2,0);
	l = new QLabel(tr("Put in variable"), d);	g->addWidget(l,2,1);
	nm = new QSpinBox(d);	nm->setRange(2,8192);	g->addWidget(nm,3,0);
	id = new QLineEdit(d);	nm->setSingleStep(10);	g->addWidget(id,3,1);
	b = new QPushButton(tr("Cancel"), d);	g->addWidget(b,4,0);
	connect(b, SIGNAL(clicked()), d, SLOT(reject()));
	b = new QPushButton(tr("OK"), d);		g->addWidget(b,4,1);
	connect(b, SIGNAL(clicked()), d, SLOT(accept()));	b->setDefault(true);
	// now execute dialog and get values
	bool res = d->exec();
	if(res && !v1->text().isEmpty() && !v2->text().isEmpty() && !id->text().isEmpty())
	{
		mglData *vv = dynamic_cast<mglData*>(parser.AddVar(id->text().toStdString().c_str()));
		if(vv)	vv->Set(mgl_data_hist(var, nm->value(), v1->text().toDouble(), v2->text().toDouble(),0));
		updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::first()	{	setSlice(0);	}
//-----------------------------------------------------------------------------
void DatPanel::last()	{	setSlice(nz-1);	}
//-----------------------------------------------------------------------------
void DatPanel::next()	{	setSlice(kz+1);	}
//-----------------------------------------------------------------------------
void DatPanel::prev()	{	setSlice(kz-1);	}
//-----------------------------------------------------------------------------
void DatPanel::gosl()
{
	bool ok;
	QString s = QInputDialog::getText(this, tr("UDAV - Go to slice"), tr("Enter slice id:"), QLineEdit::Normal, "0", &ok);
	if(ok)	setSlice(s.toInt());
}
//-----------------------------------------------------------------------------
void DatPanel::setNz(int nz)	{	sb->setMaximum(nz-1);	}
//-----------------------------------------------------------------------------
bool DatPanel::sizesDialog(const QString &cap, const QString &lab, const QString &desc1, const QString &desc2, const QString &desc3, QString &val1, QString &val2, QString &val3)
{
	QLabel *l;
	QLineEdit *f1, *f2, *f3;
	QPushButton *b;
	QDialog *d = new QDialog(this);
	d->setWindowTitle(cap);
	QVBoxLayout *v = new QVBoxLayout(d);
	l = new QLabel(lab, d);	v->addWidget(l);
	l = new QLabel(tr("NOTE: All fields must be filled!"), d);	v->addWidget(l);
	QGridLayout *g = new QGridLayout();	v->addLayout(g);
	l = new QLabel(desc1, d);		g->addWidget(l, 0, 0);
	l = new QLabel(desc2, d);		g->addWidget(l, 0, 1);
	l = new QLabel(desc3, d);		g->addWidget(l, 0, 2);
	f1 = new QLineEdit(val1, d);	g->addWidget(f1, 1, 0);
	f2 = new QLineEdit(val2, d);	g->addWidget(f2, 1, 1);
	f3 = new QLineEdit(val3, d);	g->addWidget(f3, 1, 2);
	QHBoxLayout *h = new QHBoxLayout();	v->addLayout(h);
	h->addStretch(1);
	b = new QPushButton(tr("Cancel"), d);	h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(reject()));
	b = new QPushButton(tr("OK"), d);		h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(accept()));
	b->setDefault(true);
	// now execute dialog and get values
	bool res = d->exec();
	val1 = f1->text();	val2 = f2->text();	val3 = f3->text();
	if(val1.isEmpty() || val2.isEmpty() || val3.isEmpty())	res = false;
	delete d;
	return res;
}
//-----------------------------------------------------------------------------
#include "xpm/size.xpm"
#include "xpm/crop.xpm"
#include "xpm/squize.xpm"
#include "xpm/hist.xpm"
#include "xpm/oper_dir.xpm"
#include "xpm/oper_of.xpm"
//-----------------------------------------------------------------------------
void DatPanel::newdat()
{
	QLabel *l;
	QLineEdit *f1, *f2;
	QPushButton *b;
	QDialog *d = new QDialog(this);
	d->setWindowTitle(tr("UDAV - make new data"));
	QVBoxLayout *v = new QVBoxLayout(d);
	QComboBox *c = new QComboBox(d);	v->addWidget(c);
	c->addItem(tr("Sum along direction(s)"));
	c->addItem(tr("Min along direction(s)"));
	c->addItem(tr("Max along direction(s)"));
	c->addItem(tr("Momentum along 'x' for function"));
	c->addItem(tr("Momentum along 'y' for function"));
	c->addItem(tr("Momentum along 'z' for function"));
	c->setCurrentIndex(0);

	f1 = new QLineEdit("z",d);	v->addWidget(f1);
	QCheckBox *cb = new QCheckBox(tr("Put into this data array"), d);	v->addWidget(cb);
	l = new QLabel(tr("or enter name for new variable"), d);	v->addWidget(l);
	f2 = new QLineEdit(d);		v->addWidget(f2);
	QHBoxLayout *h = new QHBoxLayout();	v->addLayout(h);	h->addStretch(1);
	b = new QPushButton(tr("Cancel"), d);	h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(reject()));
	b = new QPushButton(tr("OK"), d);		h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(accept()));
	b->setDefault(true);
	// now execute dialog and get values
	bool res = d->exec();
	QString 	val = f1->text(), mgl;
	int k = c->currentIndex();
	QString self = QString::fromStdWString(var->s);
	if(res)
	{
		if(k<0)
		{
			QMessageBox::warning(d, tr("UDAV - make new data"),
				tr("No action is selected. Do nothing."));
			return;
		}
		if(val.isEmpty())
		{
			QMessageBox::warning(d, tr("UDAV - make new data"),
				tr("No direction/formula is entered. Do nothing."));
			return;
		}
		if(cb->isChecked())	k += 6;
		QString name = f2->text();
		switch(k)
		{
		case 0:	mgl = "sum "+name+" "+self+" '"+val+"'";	break;
		case 1:	mgl = "min "+name+" "+self+" '"+val+"'";	break;
		case 2:	mgl = "max "+name+" "+self+" '"+val+"'";	break;
		case 3:	mgl = "momentum "+name+" "+self+" 'x' '"+val+"'";	break;
		case 4:	mgl = "momentum "+name+" "+self+" 'y' '"+val+"'";	break;
		case 5:	mgl = "momentum "+name+" "+self+" 'z' '"+val+"'";	break;
		case 6:	mgl = "copy "+self+" {sum "+self+" '"+val+"'}";	break;
		case 7:	mgl = "copy "+self+" {min "+self+" '"+val+"'}";	break;
		case 8:	mgl = "copy "+self+" {max "+self+" '"+val+"'}";	break;
		case 9:	mgl = "copy "+self+" {momentum "+self+" 'x' '"+val+"'}";	break;
		case 10:	mgl = "copy "+self+" {momentum "+self+" 'y' '"+val+"'}";	break;
		case 11:	mgl = "copy "+self+" {momentum "+self+" 'z' '"+val+"'}";	break;
		}
	}
	if(!mgl.isEmpty())
	{
		mglGraph gr;
		parser.Execute(&gr,mgl.toStdString().c_str());
		if(k>=6)	opers += mgl+"\n";
		updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::oper()
{
	QLineEdit *f1;
	QPushButton *b;
	QDialog *d = new QDialog(this);
	d->setWindowTitle(tr("UDAV - change data"));
	QVBoxLayout *v = new QVBoxLayout(d);
	QComboBox *c = new QComboBox(d);	v->addWidget(c);
	c->addItem(tr("Fill data by formula"));
	c->addItem(tr("Transpose data with new dimensions"));
	c->addItem(tr("Smooth data along direction(s)"));
	c->addItem(tr("Summarize data along direction(s)"));
	c->addItem(tr("Integrate data along direction(s)"));
	c->addItem(tr("Differentiate data along direction(s)"));
	c->addItem(tr("Laplace transform along direction(s)"));
	c->addItem(tr("Swap data along direction(s)"));
	c->addItem(tr("Mirror data along direction(s)"));
	c->addItem(tr("Sin-Fourier transform along direction(s)"));
	c->addItem(tr("Cos-Fourier transform along direction(s)"));
	c->addItem(tr("Hankel transform along direction(s)"));
	c->addItem(tr("Sew data along direction(s)"));
	c->addItem(tr("Find envelope along direction(s)"));
	c->setCurrentIndex(0);

	f1 = new QLineEdit("z",d);	v->addWidget(f1);
	QHBoxLayout *h = new QHBoxLayout();	v->addLayout(h);	h->addStretch(1);
	b = new QPushButton(tr("Cancel"), d);	h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(reject()));
	b = new QPushButton(tr("OK"), d);		h->addWidget(b);
	connect(b, SIGNAL(clicked()), d, SLOT(accept()));
	b->setDefault(true);
	// now execute dialog and get values
	bool res = d->exec();
	QString 	val = f1->text(), mgl;
	int k = c->currentIndex();
	QString self = QString::fromStdWString(var->s);
	if(res)
	{
		if(k<0)
		{
			QMessageBox::warning(d, tr("UDAV - make new data"),
				tr("No action is selected. Do nothing."));
			return;
		}
		switch(k)
		{
		case 0:	mgl = "modify "+self+" '"+val+"'";	break;
		case 1:	mgl = "transpose "+self+" '"+val+"'";	break;
		case 2:	mgl = "smooth "+self+" '"+val+"'";	break;
		case 3:	mgl = "cumsum "+self+" '"+val+"'";	break;
		case 4:	mgl = "integrate "+self+" '"+val+"'";	break;
		case 5:	mgl = "diff "+self+" '"+val+"'";	break;
		case 6:	mgl = "diff2 "+self+" '"+val+"'";	break;
		case 7:	mgl = "swap "+self+" '"+val+"'";	break;
		case 8:	mgl = "mirror "+self+" '"+val+"'";	break;
		case 9:	mgl = "sinfft "+self+" '"+val+"'";	break;
		case 10:	mgl = "cosfft "+self+" '"+val+"'";	break;
		case 11:	mgl = "hankel "+self+" '"+val+"'";	break;
		case 12:	mgl = "sew "+self+" '"+val+"'";	break;
		case 13:	mgl = "envelop "+self+" '"+val+"'";	break;
		}
	}
	if(!mgl.isEmpty())
	{
		mglGraph gr;
		parser.Execute(&gr,mgl.toStdString().c_str());
		opers += mgl+"\n";
		updateDataItems();
	}
}
//-----------------------------------------------------------------------------
void DatPanel::toolTop(QBoxLayout *l)
{
	QAction *a;
	QMenu *o;
	QToolButton *bb;

	// file menu
	o = menu->addMenu(tr("File"));
	a = new QAction(QPixmap(":/png/document-open.png"), tr("Load data"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(load()));
	a->setToolTip(tr("Load data from file. Data will be deleted only\nat exit but UDAV will not ask to save it (Ctrl+Shift+O)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_O);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(":/png/document-save.png"), tr("Save data"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(save()));
	a->setToolTip(tr("Save data to a file (Ctrl+Shift+S)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_S);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

//	o->addSeparator();	bb->addSeparator();
//	a = new QAction(QPixmap(insert_xpm), tr("Insert as list"), this);
//	connect(a, SIGNAL(triggered()), this, SLOT(list()));
//	o->addAction(a);
//	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);


	a = new QAction(QPixmap(":/png/office-chart-line.png"), tr("Plot data"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(plot()));
	a->setToolTip(tr("Plot data in new script window. You may select the kind\nof plot, its style and so on."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(":/png/edit-copy.png"), tr("Copy data"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(copy()));
	a->setToolTip(tr("Copy range of numbers to clipboard (Ctrl+Shift+C)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_C);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(":/png/edit-paste.png"), tr("Paste data"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(copy()));
	a->setToolTip(tr("Paste range of numbers from clipboard (Ctrl+Shift+P)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_V);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	// navigation menu
	o = menu->addMenu(tr("Navigate"));
	a = new QAction(QPixmap(":/png/go-first.png"), tr("First slice"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(first()));
	a->setToolTip(tr("Go to the first data slice for 3D data."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(":/png/go-previous.png"), tr("Prev. slice"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(prev()));
	a->setToolTip(tr("Go to the previous data slice for 3D data."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	sb = new QSpinBox(this);
	l->addWidget(sb);	sb->setRange(0,0);
	sb->setToolTip(tr("Go to the specified data slice for 3D data."));
	connect(sb, SIGNAL(valueChanged(int)), this, SLOT(setSlice(int)));
	connect(this, SIGNAL(sliceChanged(int)), sb, SLOT(setValue(int)));
	connect(this, SIGNAL(nzChanged(int)), this, SLOT(setNz(int)));

	a = new QAction(tr("Go to slice"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(gosl()));
	a->setToolTip(tr("Go to the specified data slice for 3D data."));
	o->addAction(a);

	a = new QAction(QPixmap(":/png/go-next.png"), tr("Next slice"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(next()));
	a->setToolTip(tr("Go to the next data slice for 3D data."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(":/png/go-last.png"), tr("Last slice"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(last()));
	a->setToolTip(tr("Go to the last data slice for 3D data."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);
}
//-----------------------------------------------------------------------------
void DatPanel::toolLeft(QBoxLayout *l)
{
	QAction *a;
	QMenu *o;
	QToolButton *bb;

	// size menu
	o = menu->addMenu(tr("Sizes"));
	a = new QAction(QPixmap(":/png/document-new.png"), tr("Create new"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(create()));
	a->setToolTip(tr("Recreate the data with new sizes and fill it by zeros (Ctrl+Shift+N)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_N);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(size_xpm), tr("Resize"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(reSize()));
	a->setToolTip(tr("Resize (interpolate) the data to specified sizes (Ctrl+Shift+R)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_R);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(squize_xpm), tr("Squeeze"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(squize()));
	a->setToolTip(tr("Keep only each n-th element of the data array."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(crop_xpm), tr("Cro&p"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(crop()));
	a->setToolTip(tr("Crop the data edges. Useful to cut off the zero-filled area."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(oper_of_xpm), tr("Transform"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(newdat()));
	a->setToolTip(tr("Transform data along dimension(s) (Ctrl+Shift+T)."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_T);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(oper_dir_xpm), tr("Make new (Ctrl+Shift+M)"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(oper()));
	a->setToolTip(tr("Make another data."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_M);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

	a = new QAction(QPixmap(hist_xpm), tr("Histogram (Ctrl+Shift+H)"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(hist()));
	a->setToolTip(tr("Find histogram of data."));
	a->setShortcut(Qt::CTRL+Qt::SHIFT+Qt::Key_H);	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);

/*	a = new QAction(QPixmap(":/png/view-refresh.png"), tr("Refresh"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(refresh()));
	a->setToolTip(tr("Refresh data values."));
	o->addAction(a);
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);*/

/*	a = new QAction(tr("Rearrange"), this);	// TODO: move in generalized dialog
	connect(a, SIGNAL(triggered()), this, SLOT(rearrange()));
	a->setToolTip(tr("Rearrange data sizes without changing data values."));
	o->addAction(a);
	a = new QAction(tr("Fill in range"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(inrange()));
	a->setToolTip(tr("Fill data equidistantly from one value to another."));
	o->addAction(a);
	a = new QAction(tr("Normalize"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(norm()));
	a->setToolTip(tr("Normalize data so that its minimal\nand maximal values be in specified range."));
	o->addAction(a);
	a = new QAction(tr("Norm. slices"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(normsl()));
	a->setToolTip(tr("Normalize each data slice perpendicular to some direction\nso that its minimal and maximal values be in specified range."));
	o->addAction(a);*/

	l->addStretch(1);

	a = new QAction(QPixmap(":/png/tab-close.png"), tr("Close tab"), this);
	connect(a, SIGNAL(triggered()), this, SLOT(close()));
	a->setToolTip(tr("Close this data tab."));
	bb = new QToolButton(this);	l->addWidget(bb);	bb->setDefaultAction(a);
}
//-----------------------------------------------------------------------------
QString DatPanel::dataName()	{	return QString::fromStdWString(var->s);	}
//-----------------------------------------------------------------------------
