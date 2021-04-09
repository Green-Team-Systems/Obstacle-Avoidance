import datetime 
   
# Function to convert string to datetime 
def convert_str_to_datetime(date_time): 
    format = '%m/%d/%y %H:%M:%S' # The format 
    datetime_str = datetime.datetime.strptime(date_time, format) 
   
    return datetime_str

def convert_datetime_to_str(date_time):
    format = '%m/%d/%y %H:%M:%S' # The format
    datetime_obj = date_time.strftime(format)

    return datetime_obj