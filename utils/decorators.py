def user_defined_function(func, mission):
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
    return wrapper