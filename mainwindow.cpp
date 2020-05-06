#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>

///TOTO JE DEMO PROGRAM... NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ///tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    //ipaddress="192.168.1.14";
    ipaddress="127.0.0.1";
    ui->setupUi(this);
    datacounter=0;
    lD4R.minDcrit = robotshell.Diameter;
    lD4R.DcritR = lD4R.DcritL = (3/2)*robotshell.Diameter/(sin(45.0));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem lidar len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero,pero2,pero3;

    pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena

    pero2.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero2.setWidth(3);//hrubka pera -3pixely
    pero2.setColor(Qt::red);//farba je zelena

    pero3.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero3.setWidth(3);//hrubka pera -3pixely
    pero3.setColor(Qt::magenta);//farba je magenta
    QRect rect;//(20,120,700,500);
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit



    painter.drawRect(rect);//vykreslite stvorec
    if(updateLaserPicture==1)
    {

        mutex.lock();//lock.. idem robit s premennou ktoru ine vlakno moze prepisovat...
        updateLaserPicture=0;

        painter.setPen(pero);
        ///teraz sa tu kreslia udaje z lidaru. ak chcete, prerobte
        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
        {
           ///najdenie bodu s najmensiou vzdialenostou
           if(k == 0){
                lD4R.minDist = HUGE_VAL;
                lD4R.minAngle = HUGE_VAL;
                lD4R.DistL = HUGE_VAL;
                lD4R.AngleL = HUGE_VAL;
                lD4R.DistR = HUGE_VAL;
                lD4R.AngleR = HUGE_VAL;
                lD4R.maxDistL = -HUGE_VAL;
                lD4R.maxAngleL = -HUGE_VAL;
                lD4R.maxDistR = -HUGE_VAL;
                lD4R.maxAngleR = -HUGE_VAL;
               // lD4R.minDist = copyOfLaserData.Data[k].scanDistance; //dufam ze v mm
               // lD4R.minAngle = copyOfLaserData.Data[k].scanAngle;
           }
            ///mindist
           if ((copyOfLaserData.Data[k].scanDistance < lD4R.minDist && copyOfLaserData.Data[k].scanDistance < 600.0 ) && ((360.0 - fmod(copyOfLaserData.Data[k].scanAngle,360.0)) < 25.0 || fmod(copyOfLaserData.Data[k].scanAngle,360.0) < 25.0 )){
                lD4R.minDist = copyOfLaserData.Data[k].scanDistance;
                lD4R.minAngle = copyOfLaserData.Data[k].scanAngle;
           }
            ///distL
           if(45.0 < fmod(copyOfLaserData.Data[k].scanAngle,360.0) && fmod(copyOfLaserData.Data[k].scanAngle,360.0) < 46.0 && copyOfLaserData.Data[k].scanDistance <= lD4R.DcritL){
                lD4R.DistL = copyOfLaserData.Data[k].scanDistance;
                lD4R.AngleL = copyOfLaserData.Data[k].scanAngle;
           }
            ///distR
           if(45.0 < 360 - fmod(copyOfLaserData.Data[k].scanAngle,360.0) && 360 - fmod(copyOfLaserData.Data[k].scanAngle,360.0) < 46.0 && copyOfLaserData.Data[k].scanDistance <= lD4R.DcritR){
                lD4R.DistR = copyOfLaserData.Data[k].scanDistance;
                lD4R.AngleR = copyOfLaserData.Data[k].scanAngle;
           }
            ///max dist Left
           if ((copyOfLaserData.Data[k].scanDistance > lD4R.maxDistL && copyOfLaserData.Data[k].scanDistance < 1000.0 ) && (fmod(copyOfLaserData.Data[k].scanAngle,360.0) < 60.0 )){
                lD4R.maxDistL = copyOfLaserData.Data[k].scanDistance;
                lD4R.maxAngleL = copyOfLaserData.Data[k].scanAngle;
           }
            ///max dist Right
           if ((copyOfLaserData.Data[k].scanDistance > lD4R.maxDistR && copyOfLaserData.Data[k].scanDistance < 1000.0 ) && ((360.0 - fmod(copyOfLaserData.Data[k].scanAngle,360.0)) < 60.0)){
                lD4R.maxDistR = copyOfLaserData.Data[k].scanDistance;
                lD4R.maxAngleR = copyOfLaserData.Data[k].scanAngle;
           }

            ///TODO: najst najkratsi distance
            /// zistit ci to vracia uhly zvladom na robot alebo na co - vzhladom na robot
            ///
            /// pocitat suradnice tych bodov na svetove done.
            /// spojit krajne body s najblizsim ciarou
            /// dodrziavat vzdialenost od steny ak stena stoji v ceste inak ju licovat
            //tu sa rata z polarnych suradnic na kartezske, a zaroven sa upravuje mierka aby sme sa zmestili do
            //do vyhradeneho stvorca aspon castou merania.. ale nieje to pekne, krajsie by bolo
            //keby ste nastavovali mierku tak,aby bolo v okne zobrazene cele meranie (treba najst min a max pre x a y suradnicu a podla toho to prenasobit)

           /// vykreslenie dat lidaru
            int dist=copyOfLaserData.Data[k].scanDistance/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
            int xp=rect.width()-(rect.width()/
                                 2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(rect.height() - xp, yp),2,2);//vykreslime kruh s polomerom 2px

         //   cout<< "mindist:  " << lD4R.minDist << "  DistL:  " << lD4R.DistL << "  DistR:  " << lD4R.DistR << endl;
          //  cout<< "mindistcrit:  " << lD4R.minDcrit << "  DcritL:  " << lD4R.DcritL << "  DcritR:  " << lD4R.DcritR << endl;
           //  cout<< "maxDistL:  " << lD4R.maxDistL << "  maxAngleL:  " << fmod(lD4R.maxAngleL,360) << "maxDistR:  " << lD4R.maxDistR << "  maxAngleR:  " << fmod(lD4R.maxAngleR,360) << endl;
            if(!isinf(lD4R.minX) && !isinf(lD4R.minY))
                    painter.drawEllipse(QPoint(rect.height() - xp, yp),2,2);//vykreslime kruh s polomerom 2px;
            }

        /// vypocet svetovych suradnic bodu
        if(!isinf(lD4R.minDist) && !isinf(lD4R.minAngle)){
            lD4R.forminAngle = fmod((lD4R.minAngle*M_PI/180) + fip,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
            lD4R.minX = x + ((lD4R.minDist/1000.0)*cos(lD4R.forminAngle));
            lD4R.minY = y + ((lD4R.minDist/1000.0)*sin(lD4R.forminAngle));
            lD4R.minPoint = TRUE;
        }else lD4R.minPoint = FALSE;


        /// vykreslenie bodu mindist v smere robota
        int dist=lD4R.minDist/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
        int xp=rect.width()-(rect.width()/
                         2+dist*2*sin((360.0-lD4R.minAngle)*3.14159/180.0))+rect.topLeft().x();
        int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-lD4R.minAngle)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(xp,yp))
            painter.setPen(pero2);
            painter.drawEllipse(QPoint(rect.height() - xp, yp),4,4);//vykreslime kruh s polomerom 2px


        /// vykreslenie bodu distLcrit
        int distL=lD4R.DistL/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
        int xpL=rect.width()-(rect.width()/
                         2+distL*2*sin((360.0-lD4R.AngleL)*3.14159/180.0))+rect.topLeft().x();
        int ypL=rect.height()-(rect.height()/2+distL*2*cos((360.0-lD4R.AngleL)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(xpL,ypL))
            painter.setPen(pero2);
            painter.drawEllipse(QPoint(rect.height() - xpL, ypL),4,4);//vykreslime kruh s polomerom 2px

        /// vykreslenie bodu distRcrit
        int distR=lD4R.DistR/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
        int xpR=rect.width()-(rect.width()/
                         2+distR*2*sin((360.0-lD4R.AngleR)*3.14159/180.0))+rect.topLeft().x();
        int ypR=rect.height()-(rect.height()/2+distR*2*cos((360.0-lD4R.AngleR)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(xpR,ypR))
            painter.setPen(pero2);
            painter.drawEllipse(QPoint(rect.height() - xpR, ypR),4,4);//vykreslime kruh s polomerom 2px

        /// vykreslenie bodu maxdistL
        int maxdistl=lD4R.maxDistL/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
        int maxpl=rect.width()-(rect.width()/
                         2+maxdistl*2*sin((360.0-lD4R.maxAngleL)*3.14159/180.0))+rect.topLeft().x();
        int maypl=rect.height()-(rect.height()/2+maxdistl*2*cos((360.0-lD4R.maxAngleL)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(maxpl,maypl))
            painter.setPen(pero3);
            painter.drawEllipse(QPoint(rect.height() - maxpl, maypl),4,4);//vykreslime kruh s polomerom 2px

        /// vykreslenie bodu maxdistR
        int maxdistr=lD4R.maxDistR/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
        int maxpr=rect.width()-(rect.width()/
                         2+maxdistr*2*sin((360.0-lD4R.maxAngleR)*3.14159/180.0))+rect.topLeft().x();
        int maypr=rect.height()-(rect.height()/2+maxdistr*2*cos((360.0-lD4R.maxAngleR)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(maxpr,maypr))
            painter.setPen(pero3);
            painter.drawEllipse(QPoint(rect.height() - maxpr, maypr),4,4);//vykreslime kruh s polomerom 2px

        /// vypocet svetovych suradnic bodu maxL (kraj steny lavy)
        if(!isinf(lD4R.maxDistL) && !isinf(lD4R.maxAngleL)){
            lD4R.formAngleL = fmod(lD4R.maxAngleL /180.0*M_PI + fip,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
            lD4R.maxXL = x + ((lD4R.maxDistL/1000.0)*cos(lD4R.formAngleL));
            lD4R.maxYL = y + ((lD4R.maxDistL/1000.0)*sin(lD4R.formAngleL));
            lD4R.maxPointL = TRUE;
        }else lD4R.maxPointL = FALSE;

        /// vypocet svetovych suradnic bodu maxR (kraj steny pravy)
        if(!isinf(lD4R.maxAngleR) && !isinf(lD4R.maxDistR)){
            lD4R.formAngleR = fmod(lD4R.maxAngleR/180.0*M_PI + fip,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
            lD4R.maxXR = x + ((lD4R.maxDistR/1000.0)*cos(lD4R.formAngleR));
            lD4R.maxYR = y + ((lD4R.maxDistR/1000.0)*sin(lD4R.formAngleR));
            lD4R.maxPointR = TRUE;
        }else lD4R.maxPointR = FALSE;

        //cout << "maxXL:  " << lD4R.maxXL << "  maxYL:  " << lD4R.maxYL << "  maxXR:  " << lD4R.maxXR << "  maxYR:  " << lD4R.maxYR << endl;
       // cout << "minX:  " << lD4R.minX << "  minY:  " << lD4R.minY << endl;
   ///   cout<< "minDist:  "  << lD4R.minDist << "  minAngle:  " << fmod(lD4R.minAngle,360) << "  forminAngle:  " << lD4R.forminAngle*180/M_PI << "  fip:  "  << fip*180/M_PI << "  minx  " << lD4R.minX << "  miny  " <<  lD4R.minY<< endl;
   //   cout<< "maxDistL:  "  << lD4R.maxDistL << "  maxAngleL:  " << fmod(lD4R.maxAngleL,360) << "  formAngleL:  " << lD4R.formAngleL*180/M_PI << "  fip:  "  << fip*180/M_PI << "  maxXL:  " << lD4R.maxXL << "  maxYL:  " <<  lD4R.maxYL<< endl;
   //  cout<< "maxDistR:  "  << lD4R.maxDistR << "  maxAngleR:  " << fmod(lD4R.maxAngleR,360) << "  formAngleR:  " << lD4R.formAngleR*180/M_PI << "  fip:  "  << fip*180/M_PI << "  maxXR  " << lD4R.maxXR << "  maxYR  " <<  lD4R.maxYR<< endl;
        mutex.unlock();//unlock..skoncil som
    }

}

double MainWindow::twoPoitDistance(double x1, double y1, double x2, double y2){
    double result =sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

bool MainWindow::loadTargetCoord(){
    targetX = ui->lineEdit_11->text().toDouble();
    targetY = ui->lineEdit_12->text().toDouble();
    return TRUE;
}

double MainWindow::calcAngle(double x1, double y1, double x2, double y2){
    return atan2(x2-x1,y2-y1);
}

void MainWindow::encDiff(){
    /// funckia bude vraciat rozdiel medzi pociatocnou hodnotou encodera a aktualnou
    /// taktiez treba spravit kontrolu pretecenia buffera pre encoder

    /// detekcia pretecenia encodera v oboch smeroch - lave koleso
    if(pEncL-robotdata.EncoderLeft >(60000)){
        distanceL = robot.getTick()*(robotdata.EncoderLeft-pEncL + 65535);
    }else if(pEncL-robotdata.EncoderLeft <(-60000)){
        distanceL = robot.getTick()*(robotdata.EncoderLeft-pEncL - 65535);
    }else distanceL = robot.getTick()*(robotdata.EncoderLeft - pEncL);

    /// detekcia pretecenia encodera v oboch smeroch - prave koleso
    if(pEncR-robotdata.EncoderRight >(60000)){
        distanceR = robot.getTick()*(robotdata.EncoderRight-pEncR + 65535);
    }else if(pEncR-robotdata.EncoderRight<(-60000)){
        distanceR = robot.getTick()*(robotdata.EncoderRight-pEncR - 65535);
    }else distanceR = robot.getTick()*(robotdata.EncoderRight- pEncR);

    /// vypocet uhla natocenia z odometrie
    fi = pFi + (distanceR - distanceL)/(1.0*robot.getB());
    /*if(((distanceR - distanceL)/(1.0*robot.getB())) > 0){
        fi = pFi + (distanceR - distanceL)/(1.0*robot.getB());
    }else{
        fi = pFi - (distanceR - distanceL)/(1.0*robot.getB());
    }
    */
    //fi = (fi>0 ? fi :-fi);
    // fi = fabs(fi);
    //fi =(fi*180/M_PI,360)*M_PI/180;
    fi = fmod(fi,(2*M_PI));
    ///absolutna hodnota natocenia
    if(fi < 0) fip = (2*M_PI) + fi;
    else fip = fi;

    if(distanceL == distanceR){
       // cout << "1:";
        x = x + distanceR*cos(pFi);
        y = y + distanceR*sin(pFi);
    }else{
       // cout << "0:";
        x = x + ((robot.getB()*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(sin(fi)-sin(pFi)));
        y = y - ((robot.getB()*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(cos(fi)-cos(pFi)));
    }

   // cout <<" Fideg: " << pFi*180.0/M_PI << "   " << "Fi: " << pFi << endl;
   // cout <<"x: " << x << "   " << "y: " << y << endl;

    pFi = fi;

}

void MainWindow::rotateRobot(){
  /* if(angleErr > 0) on_pushButton_6_clicked(); // Right
           else on_pushButton_5_clicked();  // Left*/
    on_pushButton_6_clicked(); //left
}

void MainWindow::angleDistFormating(){
    targetFi = calcAngle(x,y,targetX,targetY);
    targetDist = twoPoitDistance(x,y,targetX,targetY);
    if(targetFi>0.0) targetFi = (2*M_PI) - targetFi;
    else targetFi = - targetFi;
    targetFi = fmod(targetFi+M_PI_2,(2*M_PI));

    //targetFi = fmod(targetFi+90,360);
    //targetFi = targetFi*M_PI/180;
    angleErr =  targetFi -fip;
    if((targetFi > (320.0/180)*M_PI) && (fip < (40.0/180)*M_PI))
        angleErr = (2*M_PI)-angleErr;
}

void MainWindow::angleDistRegulator(){

    if(startState){
        //if(angleErr > 0 && angleErr > M_PI_4){
        if(abs(angleErr)>M_PI_4){
            rotateState = TRUE;
            rotateRobot();
        }else {
            MainWindow::on_pushButton_4_clicked();  // Stop
            rotateState = FALSE;
        }

       if(targetDist< 0.05){
            startState = FALSE;
            rotateState = FALSE;
            MainWindow::on_pushButton_11_clicked();  // Stop
            cout<< "Target reached" << endl;
        }

    }
       if(!rotateState && startState){
           /// Rozbeh po kruznici
           regData.Rcirc = regData.Krc/(angleErr);
           regData.TransSp = regData.Kts*targetDist;

           if(regData.TransSp > regData.max_trans_speed) regData.TransSp = regData.max_trans_speed;

            if(isinf(regData.Rcirc)) regData.Rcirc = 32000;
           // if(isinf(regData.Rcirc) && regData.Rcirc < 0) regData.Rcirc = -1000;

            std::vector<unsigned char> mess=robot.setArcSpeed((int)regData.TransSp,(int)regData.Rcirc);
            if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
            {

            }

             cout<< "R:  " << regData.Rcirc<< "  T:  " << regData.TransSp << "  err:  "<< angleErr << "  dist:  " << targetDist <<endl;
       }

}

void MainWindow::processThisRobot()
{
    encDiff();
    angleDistFormating();

/*
    cout<< "targetx:  " << targetX << "  targety:  " << targetY << endl;
    cout<< "targetfi:  " << targetFi*180/M_PI << "  aglerr:  " << angleErr*180/M_PI << "  targetDist:  " <<targetDist << endl;
    cout<< "x:  " << x << "  y:  "<< y << "  fi:  " << fi*180/M_PI <<  "  fip:  "  <<  fip*180/M_PI <<endl;
    cout<< "startState:  " << startState << "  rotateState:  " << rotateState << endl;
*/
    if(datacounter%2 == 0){
        angleDistRegulator();
    }

      /* if (!rotateState && startState){

           std::vector<unsigned char> mess=robot.setArcSpeed(200,20000);
           if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
           {

           }
       }*/

    ///tu mozete robit s datami z robota
     /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu len vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    if(datacounter%5==0)
    {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
   // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
    //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
    //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete


/*
    if ((robotdata.pEncL - (double)60000) > robotdata.EncoderLeft && robotdata.pEncL > robotdata.EncoderLeft ){
        //robotdata.pretecenieL += 1;
        mysig.pretecenieL += 1;
    }
    if ((robotdata.pEncR - (double)60000) > robotdata.EncoderRight && robotdata.pEncR > robotdata.EncoderRight){
        //robotdata.pretecenieR += 1;
        mysig.pretecenieR += 1;
    }
    */


        emit uiValuesChanged(mysig);
        /// toto neodporucam na nejake komplikovane struktury. robit to kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
     mutex.lock();//idem prepisovat copyOfLaserData ktoru pouziva paintEvent
     memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.

     // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
     updateLaserPicture=1;
     mutex.unlock();//skoncil som
     update();//tento prikaz je vlastne signal, ktory prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
}

void  MainWindow::setUiValues(Signal sig)
{
     ui->lineEdit_2->setText(QString::number(sig.distx));
     ui->lineEdit_3->setText(QString::number(sig.disty));
     ui->lineEdit_4->setText(QString::number(sig.robotFi));
     ui->lineEdit_5->setText(QString::number(sig.encL));
     ui->lineEdit_6->setText(QString::number(sig.encR));
     ui->lineEdit_7->setText(QString::number(sig.startL));
     ui->lineEdit_8->setText(QString::number(sig.startR));
     ui->lineEdit_9->setText(QString::number(sig.tmpPencR));
     ui->lineEdit_10->setText(QString::number(sig.tmpPencL));

}
void MainWindow::on_pushButton_11_clicked() //stop stop
{
    startState = FALSE;
    std::vector<unsigned char> mess=robot.setArcSpeed(0,0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }

}

void MainWindow::on_pushButton_10_clicked() //go on!
{
    if(loadTargetCoord()){
        targetDist = twoPoitDistance(x,y,targetX,targetY);
        targetFi = calcAngle(x,y,targetX,targetY);
       // cout << "targetX: "<< "  "<< targetX << "targetY: " << "  "<< targetY << "targetDist: " << "  " << targetDist << "targetFi: " << "  " << targetFi << endl;
        startState = TRUE;
    }else cout << "nespravny format target suradnic !" << endl;


}

void MainWindow::on_pushButton_9_clicked() //start button
{

  //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
  laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
  robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
  ///toto je prepojenie signalu o zmene udajov, na signal
  connect(this,SIGNAL(uiValuesChanged(Signal)),this,SLOT(setUiValues(Signal)));
  sleep(2);
  /// nastavenie pociatocnych hodnot encodera
  mysig.startL = startEncL = robotdata.EncoderLeft;
  mysig.startR = startEncR = robotdata.EncoderRight;
  x=y=0.0; fi=pFi=M_PI_2;  // 90 stupnov
  loadTargetCoord();
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
std::vector<unsigned char> mess=robot.setTranslationSpeed(250);

///ak by ste chceli miesto pohybu dopredu napriklad pohyb po kruznici s polomerom 1 meter zavolali by ste funkciu takto:
/// std::vector<unsigned char> mess=robot.setArcSpeed(100,1000);
if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
{

}
}

void MainWindow::on_pushButton_3_clicked() //back
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(-M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }

}


///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z lidaru (UDP komunikacia)
void MainWindow::laserprocess()
{
    // Initialize Winsock

    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    las_si_me.sin_addr.s_addr =htonl(INADDR_ANY);//moze dojst od hocikial..

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);//toto je port na ktory posielame
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    //najskor posleme prazdny prikaz
    //preco?
    //https://ih0.redbubble.net/image.74126234.5567/raf,750x1000,075,t,heather_grey_lightweight_raglan_sweatshirt.u3.jpg
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {

    }
    LaserMeasurement measure;
    while(1)
    {

        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other,&las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu
        processThisLidar(measure);

///ako som vravel,toto vas nemusi zaujimat


    }
}

///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z robota (UDP komunikacia)
void MainWindow::robotprocess()
{


    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    usleep(100*1000);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
   unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //https://i.pinimg.com/236x/1b/91/34/1b9134e6a5d2ea2e5447651686f60520--lol-funny-funny-shit.jpg
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
  //      struct timespec t;
  //      clock_gettime(CLOCK_REALTIME,&t);

        //zapamatanie predoslej hodnoty

            pEncL = robotdata.EncoderLeft;
            pEncR = robotdata.EncoderRight;

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);



        if(returnval==0)
        {
            processThisRobot();
        }


    }
}



