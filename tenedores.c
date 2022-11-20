 
// Array que representa los tenedores de los filósofos.
// 0 -> tenedor libre (sin usar)
// 1 -> tenedor ocupado (usado))
int tenedores[] = {0,0,0,0,0};

Tsemaphore sem;

void cogerTenedor(void)
{
    int tenedor0 = give_me_my_id();
    int tenedor1;
    if()
    tenedores[id] = 1;
}

void soltarTenedor(void)
{
    int id = give_me_my_id();
}

void meditar(void)
{
    int id = give_me_my_id();
    printf("\r\nMeditando %d", id);
}

void dormir(void)
{
    int id = give_me_my_id();
    printf("\r\nDurmiendo %d", id);
}

void comer(void)
{
    int id = give_me_my_id();
    printf("\r\nComiendo %d", id);
}

void proceso(void)
{
    while(1)
    {
        meditar();
        comer();
        dormir();
    }
}

