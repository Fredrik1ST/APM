
namespace APM_Running_App
{
    public partial class IntervalPage : ContentPage
    {
        int x = 2;

        public IntervalPage()
        {
            InitializeComponent();
            Dictionary<string, Entry> entries = new Dictionary<string, Entry>();
            //var text = entries["entryA"].Text;
        }

        private void Button_Clicked_Add(object sender, EventArgs e)
        {
            AddEntry(EntriesStackLayout, "Enter Interval Time (sec)");
            AddEntry(EntriesStackLayout, "Enter Rest time (sec)");
            x++;
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
            if (EntryTag1.Text == "" || EntryTag2.Text == "")
            {
                await DisplayAlert("Alert", "Please fill in all entries", "OK");
            }
            else
            {
                var response = await DisplayAlert("Start Run", "You have selected to start a run, do you want to continue?", "OK", "Cancel");
                if (response == true)
                {
                    List<string> entryValues = new List<string>();
                    string timeText_interval = "";
                    string timeText_rest = "";

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
                                    }

                                }
                            }
                        }
                        timeText_interval = entryValues.ElementAt(entryValues.Count - 2);
                        timeText_rest = entryValues.ElementAt(entryValues.Count - 1);

                        await Navigation.PushAsync(new RunStartedPage("", "", ""));
                    }
                }

            }
        }

    }
}


