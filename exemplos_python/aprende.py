import cv2 
global imagem,click,cortando
cortando = False
click = []
cam=cv2.VideoCapture(0)

def cortar(event, x, y, flags, param):

	global click,cortando,imagem

	if event == cv2.EVENT_LBUTTONDOWN:
		cortando = True
		click=[]
		click.append((x,y))

	if event == cv2.EVENT_MOUSEMOVE :
		
		if cortando:
			click.append((x,y))

	elif event == cv2.EVENT_LBUTTONUP:
		cortando = False
		cv2.rectangle(imagem, click[0], click[len(click)-1], (0, 255, 0), 2)
		

		
def exec_corte():
	global imagem,click
	clone = imagem.copy()
	cv2.namedWindow("imagem")
	cv2.setMouseCallback("imagem", cortar)


	while True:
		cv2.imshow("imagem", imagem)
		if cortando :
			print("a")
			imagem = clone.copy()
		key = cv2.waitKey(1) & 0xFF

		if cortando == True:
			#print(click)
			cv2.rectangle(imagem, click[0], click[len(click)-1], (0, 255, 0), 2)

		# se 'r' : reseta corte
		if key == ord("r"):
			imagem = clone.copy()

		# se 'c' : corta
		elif key == ord("c"):
			break


	cortada_img = clone[click[0][1]:click[len(click)-1][1], click[0][0]:click[len(click)-1][0]]
	cv2.imshow("imagem cortada", cortada_img)
	key_termina = cv2.waitKey(0)
	cv2.imwrite( "/home/borg/catkin_ws/src/robot18/ros/exemplos_python/scripts/imagens/foto_aprendida.jpg", cortada_img )

	if key_termina == 27:
		cv2.destroyAllWindows()



while True:
	ret, imagem=cam.read()
	clone = imagem.copy()

	cv2.imshow('screen',imagem)
	key_pressed = cv2.waitKey(1)

	if key_pressed==32:
		print("aqui tira foto")
		cv2.destroyAllWindows() 
		exec_corte()
		break

		
	elif key_pressed==27:
		break
	
