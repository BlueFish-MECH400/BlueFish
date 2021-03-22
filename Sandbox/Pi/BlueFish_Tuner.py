import csv

with open('BlueFish_Settings.csv') as settings_file:
    csv_reader = csv.reader(settings_file, delimiter=',')
    next(csv_reader, None)  # skip headers
    settings = {}
    for row in csv_reader:
        settings[row[0]] = row[1]

    for key in settings:
        print(key + ' = ' + settings[key])
