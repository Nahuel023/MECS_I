#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial=new QSerialPort(this);
    settingPorts=new SettingsDialog(this);
    estadoSerial = new QLabel(this);
    estadoSerial->setText("Desconectado ......");
    ui->statusbar->addWidget(estadoSerial);
    ui->actionDisconnect_Device->setEnabled(false);
    timer1=new QTimer(this);
    UdpSocket1 = new QUdpSocket(this);

    connect(ui->actionQuit,&QAction::triggered,this,&MainWindow::close);
    connect(ui->actionScanPorts, &QAction::triggered, settingPorts,&SettingsDialog::show);
    connect(ui->actionConnect_Device, &QAction::triggered,this,&MainWindow::openSerialPorts);
    connect(ui->actionDisconnect_Device, &QAction::triggered, this, &MainWindow::closeSerialPorts);
    connect(ui->pushButton_SEND,&QPushButton::clicked, this, &MainWindow::sendDataSerial);
    connect(serial,&QSerialPort::readyRead,this,&MainWindow::dataRecived);
    connect(timer1,&QTimer::timeout,this,&MainWindow::timeOut);
    connect(UdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(ui->pushButton_UDP,&QPushButton::clicked,this,&MainWindow::sendDataUDP);

    ui->comboBox_CMD->addItem("ALIVE", 0xF0);
    ui->comboBox_CMD->addItem("FIRMWARE", 0xF1);
    ui->comboBox_CMD->addItem("DS18B20", 0xA1);
    ui->comboBox_CMD->addItem("ELAPSED", 0x12);

    estadoProtocolo=START;
    rxData.timeOut=0;
    ui->pushButton_UDP->setEnabled(false);
    ui->pushButton_ALIVE->setEnabled(false);
    ui->pushButton_SEND->setEnabled(false);
    timer1->start(100);

    //Graficos
    createChartACCEL();
    createChartGYRO();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openSerialPorts(){
    const SettingsDialog::Settings p = settingPorts->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    serial->open(QSerialPort::ReadWrite);
    if (serial->isOpen()){
        ui->pushButton_ALIVE->setEnabled(true);
        ui->pushButton_SEND->setEnabled(true);
        ui->actionDisconnect_Device->setEnabled(true);
        estadoSerial->setStyleSheet("QLabel { color : blue; }");
        estadoSerial->setText(tr("Connected to %1 : %2, %3, %4, %5, %6, %7")
                                  .arg(p.name)
                                  .arg(p.stringBaudRate)
                                  .arg(p.stringDataBits)
                                  .arg(p.stringParity)
                                  .arg(p.stringStopBits)
                                  .arg(p.stringFlowControl)
                                  .arg(p.fabricante)
                              );
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
    }
}

void MainWindow::closeSerialPorts(){
    ui->actionDisconnect_Device->setEnabled(false);
    estadoSerial->setText("Desconectado ......");
    if (serial->isOpen())
        serial->close();
}

void MainWindow::dataRecived(){
    unsigned char *incomingBuffer;
    int count;

    count = serial->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    serial->read((char *)incomingBuffer,count);

    QString str="";

    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->SERIAL-->PC (" + str + ")");

    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocolo=HEADER_1;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocolo=HEADER_2;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocolo=HEADER_3;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case NBYTES:
            rxData.nBytes=incomingBuffer[i];
            estadoProtocolo=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocolo=PAYLOAD;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case PAYLOAD:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                estadoProtocolo=START;
                if(rxData.cheksum==incomingBuffer[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    ui->textEdit_RAW->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    ui->textEdit_RAW->append("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

                }
            }
            break;
        default:
            estadoProtocolo=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::decodeData(uint8_t *datosRx, uint8_t source){
    int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    QString str, strOut;
    _udat w;
    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    ui->textEdit_RAW->append("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
    case GETALIVE://     GETALIVE=0xF0,
        if(datosRx[2]==ACK){
            contadorAlive++;
            if(source)
                    str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
            else{
                    str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
            }
        }else{
            str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
        }
        ui->textEdit_PROCCES->append(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE:";
        for(uint8_t a=0;a<(datosRx[0]-1);a++){
            str += (QChar)datosRx[2+a];
        }
        ui->textEdit_PROCCES->append(str);

        break;

    case DS18B20://     Sensor de temperatura =0x10
        str = "TEMP:";
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];

        tempDS18B20 = w.i16[0];

        ui->textEdit_PROCCES->append(str + QString::number(tempDS18B20/16, 'f', 2));

        break;

    case ELAPSED:
        str = "ELAPSEDTIME:";
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        elapsedtime = w.i32;

        ui->textEdit_PROCCES->append(str + QString().number(elapsedtime ,10));
        break;

    default:
        str = str + "Comando DESCONOCIDO!!!!";
        ui->textEdit_PROCCES->append(str);
    }
}

void MainWindow::sendDataSerial(){
    uint8_t cmdId;
    //_udat   w;
    //bool ok;

    unsigned char dato[256];
    unsigned char indice=0, chk=0;

    QString str="";

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case GETALIVE://0xF0
    case GETFIRMWARE://0xF1
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;

    if(serial->isWritable()){
        serial->write(reinterpret_cast<char *>(dato),dato[NBYTES]+PAYLOAD);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }

    uint16_t valor=dato[NBYTES]+PAYLOAD;
    ui->textEdit_RAW->append("INDICE ** " +QString().number(indice,10) + " **" );
    ui->textEdit_RAW->append("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    ui->textEdit_RAW->append("CHECKSUM ** " +QString().number(chk,16) + " **" );
    ui->textEdit_RAW->append("PC--SERIAL-->MBED ( " + str + " )");

}

void MainWindow::timeOut(){
    if(rxData.timeOut){
        rxData.timeOut--;
        if(!rxData.timeOut){
            estadoProtocolo=START;
        }
    }
}

void MainWindow::on_pushButton_ALIVE_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(0);
    sendDataSerial();
}

void MainWindow::on_pushButton_OPENUDP_clicked()
{
    int Port;
    bool ok;

    if(UdpSocket1->isOpen()){
        UdpSocket1->close();
        ui->pushButton_OPENUDP->setText("OPEN UDP");
        return;
    }

    Port=ui->lineEdit_LOCALPORT->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        UdpSocket1->abort();
        UdpSocket1->bind(Port);
        UdpSocket1->open(QUdpSocket::ReadWrite);
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }
    ui->pushButton_OPENUDP->setText("CLOSE UDP");
    ui->pushButton_UDP->setEnabled(true);
    if(UdpSocket1->isOpen()){
        if(clientAddress.isNull())
            clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
        if(puertoremoto==0)
            puertoremoto=ui->lineEdit_DEVICEPORT->text().toInt();
        UdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
    }
}

void MainWindow::OnUdpRxData(){
    qint64          count=0;
    unsigned char   *incomingBuffer;

    while(UdpSocket1->hasPendingDatagrams()){
        count = UdpSocket1->pendingDatagramSize();
        incomingBuffer = new unsigned char[count];
        UdpSocket1->readDatagram( reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
    }
    if (count<=0)
        return;

    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    ui->textEdit_RAW->append(" adr " + adress);
    ui->lineEdit_IP_REMOTA->setText(RemoteAddress.toString().right((RemoteAddress.toString().length())-7));
    ui->lineEdit_DEVICEPORT->setText(QString().number(RemotePort,10));

    for(int i=0;i<count; i++){
        switch (estadoProtocoloUdp) {
        case START:
            if (incomingBuffer[i]=='U'){
                    estadoProtocoloUdp=HEADER_1;
                    rxDataUdp.cheksum=0;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                    estadoProtocoloUdp=HEADER_2;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                    estadoProtocoloUdp=HEADER_3;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                    estadoProtocoloUdp=NBYTES;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case NBYTES:
            rxDataUdp.nBytes=incomingBuffer[i];
            estadoProtocoloUdp=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                    estadoProtocoloUdp=PAYLOAD;
                    rxDataUdp.cheksum='U'^'N'^'E'^'R'^ rxDataUdp.nBytes^':';
                    rxDataUdp.payLoad[0]=rxDataUdp.nBytes;
                    rxDataUdp.index=1;
            }
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case PAYLOAD:
            if (rxDataUdp.nBytes>1){
                    rxDataUdp.payLoad[rxDataUdp.index++]=incomingBuffer[i];
                    rxDataUdp.cheksum^=incomingBuffer[i];
            }
            rxDataUdp.nBytes--;
            if(rxDataUdp.nBytes==0){
                    estadoProtocoloUdp=START;
                    if(rxDataUdp.cheksum==incomingBuffer[i]){
                    decodeData(&rxDataUdp.payLoad[0],UDP);
                    }else{
                    ui->textEdit_RAW->append(" CHK DISTINTO!!!!! ");
                    }
            }
            break;

        default:
            estadoProtocoloUdp=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::sendDataUDP(){
    uint8_t cmdId;
    //_udat w;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str;
    int puerto=0;
    //bool ok;

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case GETALIVE://0xF0
    case GETFIRMWARE://0xF1
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }

    puerto=ui->lineEdit_DEVICEPORT->text().toInt();
    puertoremoto=puerto;
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;
    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(UdpSocket1->isOpen()){
        UdpSocket1->writeDatagram(reinterpret_cast<const char *>(dato), (dato[4]+7), clientAddress, puertoremoto);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);
    ui->textEdit_RAW->append("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::on_pushButtonClearDebugger_clicked()
{
    ui->textEdit_PROCCES->clear();
    ui->textEdit_RAW->clear();
}

void MainWindow::on_pushButton_SETEO_DE_CONTROL_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(5);
    sendDataSerial();
    sendDataUDP();
}

//Control de graficas
void MainWindow::createChartACCEL()
{
    accelChart = new QChart();

    accelChart->setTitle("Valores del Aceleracion");
    accelChart->legend()->setVisible(true);
    accelChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    accelChartView = new QChartView(accelChart);

    accelChartView->setRenderHint(QPainter::Antialiasing);

    accelLayout = new QGridLayout();

    accelLayout->addWidget(accelChartView, 0, 0);

    ui->widgetAccel->setLayout(accelLayout);

    accelXSpline = new QSplineSeries();
    accelYSpline = new QSplineSeries();
    accelZSpline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        accelXDatos.append(QPointF(i, 0));
        accelYDatos.append(QPointF(i, 0));
        accelZDatos.append(QPointF(i, 0));
    }

    accelXSpline->append(accelXDatos);
    accelYSpline->append(accelYDatos);
    accelZSpline->append(accelZDatos);

    accelXSpline->setName("ACCEL X");
    accelYSpline->setName("ACCEL Y");
    accelZSpline->setName("ACCEL Z");

    accelChart->addSeries(accelXSpline);
    accelChart->addSeries(accelYSpline);
    accelChart->addSeries(accelZSpline);

    accelChart->createDefaultAxes();
    accelChart->axes(Qt::Vertical).first()->setRange(-3, 3);
    accelChart->axes(Qt::Horizontal).first()->setRange(0, 30);
}

void MainWindow::addPointChartACCELX(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelXDatos.replace(i, QPointF(i, accelXDatos.value(i + 1).ry()));
    }

    accelXDatos.removeLast();
    accelXDatos.append(QPointF(30, point * 1.0));

    accelXSpline->clear();
    accelXSpline->append(accelXDatos);
}

void MainWindow::addPointChartACCELY(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelYDatos.replace(i, QPointF(i, accelYDatos.value(i + 1).ry()));
    }

    accelYDatos.removeLast();
    accelYDatos.append(QPointF(30, point * 1.0));

    accelYSpline->clear();
    accelYSpline->append(accelYDatos);
}

void MainWindow::addPointChartACCELZ(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelZDatos.replace(i, QPointF(i, accelZDatos.value(i + 1).ry()));
    }

    accelZDatos.removeLast();
    accelZDatos.append(QPointF(30, point * 1.0));

    accelZSpline->clear();
    accelZSpline->append(accelZDatos);
}


//Control de graficas
void MainWindow::createChartGYRO()
{
    gyroChart = new QChart();

    gyroChart->setTitle("Valores del Giroscopio");
    gyroChart->legend()->setVisible(true);
    gyroChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    gyroChartView = new QChartView(gyroChart);

    gyroChartView->setRenderHint(QPainter::Antialiasing);

    gyroLayout = new QGridLayout();

    gyroLayout->addWidget(gyroChartView, 0, 0);

    ui->widgetGyro->setLayout(gyroLayout);

    gyroXSpline = new QSplineSeries();
    gyroYSpline = new QSplineSeries();
    gyroZSpline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        gyroXDatos.append(QPointF(i, 0));
        gyroYDatos.append(QPointF(i, 0));
        gyroZDatos.append(QPointF(i, 0));
    }

    gyroXSpline->append(gyroXDatos);
    gyroYSpline->append(gyroYDatos);
    gyroZSpline->append(gyroZDatos);

    gyroXSpline->setName("gyrox");
    gyroYSpline->setName("gyroy");
    gyroZSpline->setName("gyroz");

    gyroChart->addSeries(gyroXSpline);
    gyroChart->addSeries(gyroYSpline);
    gyroChart->addSeries(gyroZSpline);

    gyroChart->createDefaultAxes();
    gyroChart->axes(Qt::Vertical).first()->setRange(-250, 250);
    gyroChart->axes(Qt::Horizontal).first()->setRange(0, 30);
}

void MainWindow::addPointChartGYROX(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroXDatos.replace(i, QPointF(i, gyroXDatos.value(i + 1).ry()));
    }

    gyroXDatos.removeLast();
    gyroXDatos.append(QPointF(30, point * 1.0));

    gyroXSpline->clear();
    gyroXSpline->append(gyroXDatos);
}

void MainWindow::addPointChartGYROY(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroYDatos.replace(i, QPointF(i, gyroYDatos.value(i + 1).ry()));
    }

    gyroYDatos.removeLast();
    gyroYDatos.append(QPointF(30, point * 1.0));

    gyroYSpline->clear();
    gyroYSpline->append(gyroYDatos);
}

void MainWindow::addPointChartGYROZ(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroZDatos.replace(i, QPointF(i, gyroZDatos.value(i + 1).ry()));
    }

    gyroZDatos.removeLast();
    gyroZDatos.append(QPointF(30, point * 1.0));

    gyroZSpline->clear();
    gyroZSpline->append(gyroZDatos);
}
