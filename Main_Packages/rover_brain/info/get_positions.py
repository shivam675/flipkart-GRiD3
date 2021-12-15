import yaml



if __name__ == '__main__':
    with open('locations.yaml') as file:
        data = yaml.load(file, yaml.FullLoader)
        print(data['chutes']['mumbai']['px'])