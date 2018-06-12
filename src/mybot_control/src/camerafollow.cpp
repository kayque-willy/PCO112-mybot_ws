#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <stdio.h>

double velX = 0, velZ = 0;
int circuloEscolhido;
bool encontrou = false;

//Gira o carrinho para a direita para procurar circulos
void iniciarProcura(){
	ROS_INFO_STREAM("Nenhum circulo encontrado, procurando...");
	velZ = 0.15;
	encontrou = false;
}

//Se encontrar um circulo da um break na rotacao, o valor negativo e por causa da inercia
void finalizarProcura(){
	ROS_INFO_STREAM("Circulo encontrado!");
	velZ = -0.15;
	encontrou = true;	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		//Objetos MAT que recebem a imagem gray que e a escala em cinza e src que e a imagem raw
		cv::Mat gray, src = cv_bridge::toCvShare(msg, "bgr8")->image; //Converte a mensagem com a imagem para opencv
		//Vetor que recebe o circulo
		cv::vector<cv::Vec3f> circles;
		
		//Converte src para escala cinza em gray
		cv::cvtColor( src, gray, CV_BGR2GRAY );
		// Reduz o ruido na detecao do circulo em gray
		cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
		// Aplica a Hough Transform para encontrar circulos de gray para o vetor circles
		cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );
		
		//Com a funcao HoughCircles sao adicionados os circulos encontrados da imagem no vetor circles
		//Verifica se encontrou o circulo através da variavel de controle encontrou
		if(encontrou == false){
			//Se o vetor estiver vazio, gira o carrinho para procurar circulos no ambiente
			if(circles.size() == 0){
				//Gira o carro para direita enquanto nao encontrar nenhum circulo
				iniciarProcura();
			}else{
				//Caso encontre um circulo, da um break no giro
				finalizarProcura();
			}
			//Da um break na velocidade linear sempre que nao tiver algum circulo na tela
			velX = 0;	
		}else{
			//Este IF interno e apenas para evitar acessar o vetor vazio
			//caso o circulo nao exista e a variavel de controle ainda esteja verdadeira
			if(circles.size() != 0){
				//Aqui e feita a movimentacao em direcao ao circulo
				//As informacoes do vetor circles sao: 
				//raio: (circles[i][0]), ponto cartesiano X: (circles[i][1]), ponto cartesiano Y: (circles[i][2])
				int raio = cvRound(circles[circuloEscolhido][2]); 
				int pX = cvRound(circles[circuloEscolhido][0]);
				
				//Vai em direcao ao circulo de acordo com o tamanho do raio
				//O valor 230 foi escolhido por corresponder a uma esfera que esta proxima a camera
				//Enquanto o raio nao for este valor arbitrario, move o carrinho em direcao do circulo
				if(raio < 230){
					ROS_INFO_STREAM("Circulo encontrado, indo em direcao.");
					//Faz as correcoes para manter o carrinho direcionado para o ciculo
					//Os valores 440 e 370 tambem sao arbitrarios e correspondem a posicao horizontal do circulo na tela
					//Se o circulo se encontra entre essa faixa de valores ele esta proximo ao centro da tela
					//Correcao para a direita
					if(pX > 440){
						velZ = 0.1;
						velX = 0.1;
					//Correcao para esquerda
					}else if(pX < 370){
						velZ = -0.1;
						velX = 0.1;
					//Segue reto
					}else{
						velZ = 0;
						velX = 0.2;
					}
				}else{
					ROS_INFO_STREAM("Circulo encontrado, chegou.");
					velZ = 0;
					velX = 0;
				}
			//Este IF interno e apenas para evitar acessar o vetor vazio 
			//caso o circulo nao exista e a variavel de controle ainda esteja verdadeira
			//Se ocorrer essa inconsistencia, inicia a procura novamente
			}else{
				iniciarProcura();
			}
		}
		
		// Desenha os circulos detectados em src 
		for( size_t i = 0; i < circles.size(); i++ ){
			//Ponto central do circulo
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			//Raio do circulo
			int raio = cvRound(circles[i][2]);     
			//Desenha o circulo em src para exibicao na tela
			cv::circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// circulo verde pequeno do centro     
			cv::circle(src, center, raio, cv::Scalar(0,0,255), 3, 8, 0 );// circulo vermelho ao redor
			ROS_INFO_STREAM("\nCirculo: " << (i + 1) << "\nCentro : " << center << "\nRaio : [" << raio << "]" << std::endl);
		}
		
		// Mostra os resultados no visualizador de imagem
		cv::imshow("View", src);
		cv::waitKey(30);
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Nao e possivel converter de '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "camerasub");
	ros::NodeHandle nh;
	geometry_msgs::Twist msg;
	
	//Publisher de velocidade
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mybot/cmd_vel", 10);
	
	//Subscribe da imagem da camera
	cv::namedWindow("View", CV_WINDOW_NORMAL);
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/mybot/camera1/image_raw", 1, &imageCallback);
	
	ros::Rate rate(3);
	while (ros::ok()){
			ros::spinOnce();	
			msg.linear.x = velX;
			msg.angular.z = velZ;
			ROS_INFO_STREAM("\nVelocidade: Angular: [" << velZ << "] Linear: [" << velX << "]");
			pub.publish(msg);
			rate.sleep();
	}
	
	cv::destroyWindow("View");
	
	return 0;
}
