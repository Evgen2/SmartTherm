/* SD_filter.hpp  */
#ifndef SD_FILTER
#define SD_FILTER

class fast_safe_filter
{
public:

    float last_confirmed; // Последнее надежное значение
    float prev_confirmed; // Предоследнее надежное значение
    int suspect_count;    // Счетчик "подозрительных" измерений подряд
    float suspect_diff;    //
    int is_initialized;
    float Max_Gap;         // Макс. изменение между измереними, чтобы не считать подозрительными
    int Adapt_Limit;       // Через сколько шагов поверить в резкий скачок

	fast_safe_filter(void)
	{	 //MAX_GAP = 3.5f;
		 //ADAPT_LIMIT = 2;      
		last_confirmed = prev_confirmed = 0.0f;
		suspect_count = 0;
		suspect_diff = 0.0f;
		is_initialized = 0;
        Adapt_Limit = 2;
        Max_Gap = 3.5f;
	}

	float filter(float next_val)
    {   // Инициализация при первом включении
        if (!is_initialized) {
            last_confirmed = prev_confirmed= next_val;
            is_initialized = 1;
            return next_val;
        }

    // Вычисляем отклонение от последнего надежного значения
        float diff = next_val - last_confirmed;
        if (diff < 0) diff = -diff;

        if (diff <= Max_Gap) {
            // --- СЛУЧАЙ А: Значение в пределах нормы ---
            prev_confirmed = last_confirmed; // Сохраняем предыдущее надежное значение
            last_confirmed = next_val; // Обновляем опору
            suspect_count = 0;         // Сбрасываем счетчик подозрений
            suspect_diff = 0.0f;
            return next_val;           // Выдаем мгновенно (задержка = 0)
        } else {
            // --- СЛУЧАЙ Б: Подозрение на помеху ---
            suspect_count++;
            suspect_diff += diff;

            if (suspect_count >=  Adapt_Limit) {
                // Если сигнал изменился и держится (реальный скачок)
                if(suspect_diff >  Max_Gap * suspect_count && diff < (Max_Gap * suspect_count*2))
                {	prev_confirmed = last_confirmed; 
                    last_confirmed = next_val;
                    suspect_count = 0;
                    suspect_diff = 0.0f;
                    return next_val;
                } else if(suspect_count >= Adapt_Limit * 2) {
                    prev_confirmed = last_confirmed;
                    last_confirmed = (last_confirmed + next_val)/2.;
                    last_confirmed = next_val;
                    suspect_count = 0;
                    suspect_diff = 0.0f;
                    return next_val;
                }
            }
            // Пока считаем помехой — возвращаем старое доброе значение
            return last_confirmed;
        }
    }
};

#endif // SD_FILTER
