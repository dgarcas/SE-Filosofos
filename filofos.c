// Array que representa los tenedores de los filósofos.
// 0 -> tenedor libre (sin usar)
// 1 -> tenedor ocupado (usado))
/*int tenedores[] = {0,0,0,0,0};
Tsemaphore sem;

void proceso(void)
{
    while(1)
    {
        meditar();
        comer();
        dormir();
    }
}


void meditar(void)
{
    int id = give_me_my_id();
    printf("\r\nMeditando %d", id);
    
}

void comer(void)
{
    int id = give_me_my_id();
    cogerTenedor();
    printf("\r\nComiendo %d", id);
    soltarTenedor();
}

void dormir(void)
{
    int id = give_me_my_id();
    printf("\r\nDurmiendo %d", id);
}


void cogerTenedor(void)
{
    int tenedorA = give_me_my_id();
    int tenedorB = getIdTenedorB (); 
    
    wait(&sem);
    if (tenedores[tenedorA] == 0 && tenedores[tenedorB] == 0){
        tenedores[tenedorA] = 1; 
        tenedores[tenedorB] = 1; 
    }
    signal(&sem);
}

int getIdTenedorB(void){
    int myId = give_me_my_id();
    int tenedorId;
    if(myId == 4){
        tenedorId = 0;
    }else {
        tenedorId = myId + 1;
    }
}

void soltarTenedor(void)
{
    int tenedorA = give_me_my_id();
    int tenedorB = getIdTenedorB (); 
    
    wait(&sem);
    if (tenedores[tenedorA] == 1 && tenedores[tenedorB] == 1){
        tenedores[tenedorA] = 0; 
        tenedores[tenedorB] = 0; 
    }
    signal(&sem);
}*/