#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QStackedLayout>

#include "graph.h"
#include "vis_widget.h"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  
  graph_t graph("/home/sa1nt/temp/graph/stpeter.graph");

//  QWidget mainWidget;
  
//  QVBoxLayout * mainLayout = new QVBoxLayout(&mainWidget);
  
  VisWidget visual;// = new VisWidget(&mainWidget);
  visual.setGraph(graph);  
  visual.show();
  

  QWidget panel;// = new QWidget;//(&mainWidget);

  QVBoxLayout * layout = new QVBoxLayout(&panel);  
  
  QPushButton * btnDijkstra = new QPushButton("Dijkstra", &panel);
  QPushButton * btnBidiDijkstra = new QPushButton("Bidirectional Dijkstra", &panel);
  QPushButton * btnAStar = new QPushButton("A star", &panel);
  QPushButton * btnBidiAStar = new QPushButton("Bidirectional AStar", &panel);
  
  layout->addWidget(btnDijkstra);
  layout->addWidget(btnBidiDijkstra);
  layout->addWidget(btnAStar);
  layout->addWidget(btnBidiAStar);
  
  panel.setLayout(layout);

  QObject::connect(btnDijkstra, SIGNAL(pressed()),
                   &visual, SLOT(runDijkstra()));
  QObject::connect(btnBidiDijkstra, SIGNAL(pressed()),
                   &visual, SLOT(runBidiDijkstra()));
  QObject::connect(btnAStar, SIGNAL(pressed()),
                   &visual, SLOT(runAStar()));
  QObject::connect(btnBidiAStar, SIGNAL(pressed()),
                   &visual, SLOT(runBidiAStart()));
        

//  mainLayout->addWidget(visual);
//  mainLayout->addWidget(panel);

//  mainWidget.setLayout(mainLayout);
//  mainWidget.show();
  panel.show();
  visual.show();
  return app.exec();
}
