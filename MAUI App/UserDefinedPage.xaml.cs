
// Some of this code is from : https://learn.microsoft.com/en-us/dotnet/maui/user-interface/controls/entry?view=net-maui-8.0

//using Android.Widget;
using System.Diagnostics.Metrics;

namespace APM_Running_App
{
    public partial class UserDefinedPage : ContentPage
    {
        int x = 2;


        public UserDefinedPage()
        {
            InitializeComponent();
            Dictionary<string, Entry> entries = new Dictionary<string, Entry>();
        }

        private void Button_Clicked_Add(object sender, EventArgs e)
        {
            AddEntry(EntriesStackLayout, "Enter distance in segment (m)");
            AddEntryTime(EntriesStackLayout, "min", "sec");
            x++;
        }

        private void AddEntryTime(StackLayout sl, string name1, string name2)
        {
            StackLayout mainLayout = new StackLayout
            {
                Orientation = StackOrientation.Horizontal,
                VerticalOptions = LayoutOptions.Center
            };

            Label label = new Label
            {
                Text = x.ToString() + ": ",
                VerticalOptions = LayoutOptions.Center,
                TextColor = Color.FromArgb("#0B90FF")
            };


            // Create the first entry with minutes
            Entry entryTime1 = new Entry
            {
                Placeholder = name1,
                Keyboard = Keyboard.Numeric,
            };

            // Create the label ":"
            Label colonLabel = new Label
            {
                Text = ":",
                VerticalOptions = LayoutOptions.Center,
                HorizontalOptions = LayoutOptions.Center
            };

            // Create the second entry with seconds
            Entry entryTime2 = new Entry
            {
                Placeholder = name2,
                Keyboard = Keyboard.Numeric,
            };

            //// Create the grid
            //Grid grid = new Grid();
            //grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            //grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            //grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

            // Add the entries and label to the grid
            //Grid.SetColumn(entryTime1, 0);

            mainLayout.Children.Add(label);
            mainLayout.Children.Add(entryTime1);

           //Grid.SetColumn(colonLabel, 1);
            mainLayout.Children.Add(colonLabel);

            //Grid.SetColumn(entryTime2, 2);
            mainLayout.Children.Add(entryTime2);

            // Add the grid to the main stack layout
            sl.Children.Add(mainLayout);
        }


        private void AddEntry(StackLayout sl, string name)
        {
            StackLayout entryLayout = new StackLayout
            {
                Orientation = StackOrientation.Horizontal,
                VerticalOptions = LayoutOptions.Center
            };

            Label label = new Label
            {
                Text = x.ToString() + ": ",
                VerticalOptions = LayoutOptions.Center,
                TextColor = Color.FromArgb("#0B90FF")
            };

            Entry entry = new Entry()
            {
                Placeholder = name,
                VerticalOptions = LayoutOptions.Center,
            };
            entryLayout.Children.Add(label);
            entryLayout.Children.Add(entry);

            // Add the entry layout to the main stack layout
            sl.Children.Add(entryLayout);

            //sl.Children.Add(label);
            //sl.Children.Add(entry);
        }

        private void RemoveEntry()
        {
            if (EntriesStackLayout.Children.Count > 0)
            {
                EntriesStackLayout.Children.RemoveAt(EntriesStackLayout.Children.Count - 1);
            }
        }

        private void Button_Clicked_Remove(object sender, EventArgs e)
        {
            if (EntriesStackLayout.Children.Count > 1)
            {
                RemoveEntry();
                RemoveEntry();
                x--;
            }
        }
        async void Button_Clicked(object sender, EventArgs e)
        {
            if (entry_time1.Text == " " || entry_time2.Text == " ")
            {
                await DisplayAlert("Alert", "Please fill in all entries", "OK");
            }
            else
            {
                var response = await DisplayAlert("Start Run", "You have selected to start a run, do you want to continue?", "OK", "Cancel");
                if (response == true)
                {
                    List<string> entryValues = new List<string>();
                    string distText = "";
                    string timeText_min = "";
                    string timeText_sec = "";

                    List<int> dist_segments = new List<int>();
                    List<int> time_segments = new List<int>();

                    if (EntriesStackLayout.Children.Count > 1)
                    {
                        foreach (var child in EntriesStackLayout.Children)
                        {
                            if (child is StackLayout stackLayout)
                            {
                                foreach (var subChild in stackLayout.Children)
                                {
                                    if (subChild is Entry entry)
                                    {
                                        entryValues.Add(entry.Text);

                                        //if (counter % 2 == 0) // Every third value
                                        //{

                                        //}
                                        //else
                                        //{
                                        //    time_segments.Add(entry.Text);
                                        //}
                                        //counter++;
                                    }

                                }
                            }
                        }
                        distText = entryValues.ElementAt(entryValues.Count - 3);
                        timeText_min = entryValues.ElementAt(entryValues.Count - 2);
                        timeText_sec = entryValues.ElementAt(entryValues.Count - 1);
                    }
                    else
                    {
                        // Get the text from the Entry controls directly
                        distText = EntryTag1.Text;
                        timeText_min = entry_time1.Text;
                        timeText_sec = entry_time2.Text;
                    }

                    await Navigation.PushAsync(new RunStartedPage(distText, timeText_min, timeText_sec));
                }
            }
        }
    }



}


