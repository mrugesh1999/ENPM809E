=============================
The taskplanning API reference
=============================

.. automodule:: taskplanning
    :members:

initiate()
=============================

.. code-block:: python

   def initiate():
       """
       A function to initiate all dictionaries
       :return: Initiate status dictionaries
       """



pick()
=============================


.. code-block:: python

   def pick(arm, part):
       """
       A function to pick the part
       :param arm: The arm is either Left or Right accepted in str format
       :param part: The initial of the color of the part in str format
       :return: Changes the status variables accordingly
       """



place()
=============================


.. code-block:: python

    """
    A function to place the part in tray
    :param arm: The arm is either Left or Right accepted in str format
    :param part: The initial of the color of the part in str format
    :return: Changes the status variables accordingly
    """



get_user_data()
=============================

.. code-block:: python

   def get_user_data():
       """
       A function to get data from the user
       :return: Get user data and store it into variables
       """


move_to_bin()
=============================

.. code-block:: python

   def move_to_bin():
       """
       This function will move robot to the bin
       :return: Change variables accordingly
       """



move_to_tray()
=============================

.. code-block:: python

   def move_to_tray():
       """
       This function will move robot to the tray
       :return: Change variables accordingly
       """


compute()
=============================

.. code-block:: python

   def compute():
       """
       The function that calls all the previous function
       :return: Print the plan
       """



=============================
The taskexecution API reference
=============================

.. automodule:: taskplanning
    :members:

Execution file
=============================

.. code-block:: python

   # Importing the file with all the functions
   import taskplanning

   # Calling the final function
   taskplanning.compute()
