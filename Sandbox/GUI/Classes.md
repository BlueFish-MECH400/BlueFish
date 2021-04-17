# Classes

## FishCommand_main.py

### FishCommandWindow

```python
def connect_buttons(self):
```
* Binds each button in the GUI to their corresponding methods

```python
def set_combobox_data(self):
```
* Provide data values for combo boxes (dropdown menus) with units in text

```python
def save_settings(self):
```
* Save csv with all metadata and settings

```python
def load_settings(self):
```
* Allow user to choose csv file and load bluefish settings into GUI
* Load csv with all metadata and settings

```python
def get_bluefish_settings(self) -> None:
```
* Update the settings dictionary with currents settings in the FishCommand UI
* Returns nothing

```python
def set_bluefish_settings(self) -> None:
```
* Set FishCommand UI values to those from the saved settings
* Returns nothing

```python
def push_settings_to_bluefish(self):
```
* Get user input settings, interrupt arduino program to update arduino operational settings

```python
def start_logging(self, filepath):
```
* Start a logging thread and connect all signals and slots
* Creates an instance from the Logger class in csv_logger.py

```python
def stop_logging(self):
```
* Stop and eliminate logging thread, setting _is_logger_running to false

```python
def get_plot_settings(self) -> None:
```
* Update plot_settings dictionary with user inputs in combo boxes (dropdown menus)

```python
def start_plotting(self, settings: dict, filepath):
```
* Start a logging thread and connect all signals and slots
